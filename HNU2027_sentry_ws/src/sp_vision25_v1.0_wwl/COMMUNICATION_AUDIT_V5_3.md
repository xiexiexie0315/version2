# v5.3 通信全量复核结论

对照对象：

- 可用导航：`pb2025_sentry_ws/src/standard_robot_pp_ros2`
- 最新电控：`newcommunication2.0/src/task/transmission`
- 自瞄：本目录的 `standard_mpc_serial + NavSerialGimbal`

## 已一致的线协议

三方基础帧定义一致：

```text
[SOF=0x5A][LEN][ID][CRC8][DATA][CRC16 low][CRC16 high]
```

- 115200、8N1、无流控。
- RobotCmd ID `0x01`，总长 49，LEN 43，固定头 `5A 2B 01 7F`。
- IMU ID `0x02`，总长 34。
- CRC8 初值 `0xFF`，覆盖前三字节。
- CRC16 初值 `0xFFFF`，反射多项式 `0x8408`，结果小端。
- RobotCmd 关键偏移：pitch 36、yaw 40、fire 44、fric 45、tracking 46、CRC16 47。
- 电控和自瞄周期均约 5 ms，即约 200 Hz。

v5.3 对总长和所有关键偏移增加了 `static_assert`，并提供 `nav_serial_wire_test` 强制测试 `0x5A/0x5B` 转义、CRC、IMU 上行和下位机字段提取；另跑 4096 组随机帧覆盖载荷与 CRC 中的转义字节。

## 不能直接照搬导航源码的地方

1. 磁盘导航源码没有字节转义，当前 `newcommunication2.0` 明确要求双向转义：除首 SOF 外，`5A -> 5B 01`，`5B -> 5B 02`。CRC 在转义前计算。因此 v5.3 保留 `nav_serial_byte_stuffing: true`。这说明磁盘导航源码、实车运行二进制和车上固件至少有一方并非同一版本。

2. 导航 ROS 合同是弧度，但当前电控云台控制链实际是度。电控接收 RobotCmd 后直接复制角度，后续 pitch 限幅 `[-30,20]`、扫描阈值 `135` 和 INS 反馈都按度，没有 rad-to-deg。v5.2 的 scale=1.0 会少 57.3 倍，v5.3 改为 `57.2957795`。

3. 电控在 GYRO 中持续刷新 `yaw_offset_angle`，切入 AUTO 后冻结；上传 yaw 已经是相对冻结基准的角度，因此 v5.3 不再二次减首帧。上行 yaw 是 `offset - INS`，PID 反馈是 `INS - offset`，两者符号相反，所以 yaw scale 为 `-57.2957795`；pitch scale 为正。轴符号仍必须通过断开发射后的单轴小角度实车确认。

4. 导航无条件发送，v5.2 在 IMU 过期后完全停发，会让没有 watchdog 的电控永久保留最后角度。v5.3 改为：连接存在时始终约 200 Hz 发送；只有 IMU 新鲜且有目标时发送有效角度，其他情况发送全零安全指令。

## v5.3 已整改

- 双向字节转义默认开启，动态 timestamp 会持续覆盖需要转义的字节值。
- Planner 弧度转电控度，yaw 同时按电控上下行坐标关系反号。
- 直接使用电控已冻结的 AUTO yaw 基准，避免程序在 AUTO 中重启时二次重定基。
- 丢目标立即清零 yaw/pitch，不保留上一次目标。
- 相机/识别链超过 200 ms 未更新时，最后一个目标自动失效，避免相机掉线后持续下发旧角度。
- 串口 TX 层另有独立 200 ms 命令 watchdog；即使 Planner 卡住，也会把旧命令变为 tracking=0 的安全零帧。
- 相机取图改为 100 ms 有界等待，相机掉线时主线程仍能响应退出，通信线程也不会保留旧目标。
- fire/fric 始终为 0。
- 下行填入单调毫秒 timestamp。
- 单次 write 改为循环写完整帧，部分写入记为错误并重连。
- 打开失败和运行中拔插均每秒自动重连。
- 重连期间停止有效控制；收到新 IMU 前只发安全零帧。
- IMU/关节数据含 NaN/Inf 时丢弃并计数。
- ID `0x0C` 不再永久覆盖用于瞄准的 IMU yaw/pitch。
- 正常退出和异常收口都会清零，并等待 TX 线程确认至少写出一帧安全零帧；不再用固定 sleep 猜测是否发出。
- 日志增加连接、超时、非法转义、CRC、无效 IMU、TX 完整帧和转义增量计数。

## 自瞄侧无法彻底解决的剩余项

- 电控没有 PC 命令超时 watchdog。Linux 突然断电/进程被 kill 时，上位机无法再发零帧。
- 电控的 yaw 基准由 `gyro_yaw_inherit` 决定，但协议没有上报当前控制模式和该基准；首次闭环必须按测试流程确认。
- 电控没有上传实时弹速，自瞄暂用配置值 22 m/s；这不影响云台是否能转，但影响弹道精度。
- 本阶段自瞄发送的底盘 vx/vy/wz 为 0，而且独占串口，不能与导航串口节点同时运行。这符合第一阶段，第三阶段需要另行设计仲裁/中间人。

结论：v5.3 的线协议比磁盘导航源码更贴合当前 `newcommunication2.0`，并已修正 v5.2 的单位、yaw 语义和串口健壮性问题。可以先做断开发射、架稳后的 0.05 rad 单轴测试；在没有电控 watchdog 前，不应直接做无人看守的完整实车运行。
