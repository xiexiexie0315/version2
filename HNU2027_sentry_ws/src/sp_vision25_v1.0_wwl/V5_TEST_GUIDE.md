# v5.3 上车编译与测试流程

本版是纯 C++ 串口自瞄，不需要 source ROS2，也不运行 ROS2 节点。它保留 v5.2 的海康连续采集修复，并补上角度单位、相对 yaw、安全零指令、串口重连、完整写入和非法上行数据防护。

## 1. 放置目录

将整个 `sp_vision25_serial_hardened_v5_3` 文件夹放到 Linux 用户目录，例如：

```text
/home/rm/sp_vision25_serial_hardened_v5_3
```

不要只拿 `src`、`io` 或单个可执行文件，模型、相机库、配置和源码必须保持原目录结构。

## 2. 运行前检查

```bash
ls -l /dev/ttyACM*
groups
ps -ef | grep -E 'standard_robot_pp|serial|bringup' | grep -v grep
```

要求：

- 电控 USB 虚拟串口确实是 `/dev/ttyACM0`；不是就修改 `configs/standard3.yaml` 的 `nav_serial_port`。
- 当前用户属于 `dialout` 组；否则执行 `sudo usermod -aG dialout $USER` 后注销并重新登录。
- 停止导航 bringup/串口节点。导航和本程序不能同时占用 `/dev/ttyACM0`。
- 第一次闭环测试物理断开发射机构，并把机器人架稳、清空云台运动范围。

## 3. 编译

```bash
cd ~/sp_vision25_serial_hardened_v5_3
rm -rf build-serial
cmake -S . -B build-serial \
  -DCMAKE_BUILD_TYPE=Release \
  -DSP_VISION_DISABLE_ROS2=ON
cmake --build build-serial --target nav_serial_wire_test standard_mpc_serial -j2
./build-serial/nav_serial_wire_test
```

协议测试必须输出类似：

```text
PASS frame=49 len=43 header=5A-2B-01-7F ... safe_crc=188D imu=34 fuzz=4096
```

真正的运行入口只有：

```text
build-serial/standard_mpc_serial
```

不要再编译或运行 `standard_mpc_ros2`。

## 4. 配置基准

`configs/standard3.yaml` 保持：

```yaml
nav_serial_port: "/dev/ttyACM0"
nav_serial_baud: 115200
nav_serial_yaw_scale: -57.2957795
nav_serial_pitch_scale: 57.2957795
nav_serial_yaw_relative_to_start: false
nav_serial_imu_timeout_ms: 500
nav_serial_target_timeout_ms: 200
nav_serial_command_timeout_ms: 200
nav_serial_reconnect_ms: 1000
nav_serial_byte_stuffing: true
nav_serial_manual_test: false
```

原因不是 ROS2 和 C++ 的区别，而是两端语义不同：导航 ROS 消息和自瞄 Planner 都用弧度；当前电控 `SBUS + WHEEL_OMNI_INFANTRY` 云台 PID、pitch 限幅 `[-30, 20]` 和扫描阈值 `135` 都按度工作，接收路径又没有 rad-to-deg 转换，因此 v5.3 在自瞄侧转换为度。电控上行 yaw 使用 `offset - INS`，PID 反馈使用 `INS - offset`，所以 yaw 还必须反号；pitch 上下行同号。

电控以后若在接收处完成“pitch 弧度转度、yaw 弧度转度并反号”，必须同时把两个 scale 改回 `1.0`，禁止两边重复转换或重复反号。

## 5. 先做纯通信和小角度测试

先在 GYRO 模式启动程序并确认 IMU 正常，再由电控切入 AUTO。电控会在 GYRO 中持续刷新 yaw offset，进入 AUTO 后冻结该基准；v5.3 不再自行减首帧 yaw，避免在 AUTO 中重启程序时产生二次重定基。

临时修改：

```yaml
nav_serial_manual_test: true
nav_serial_manual_test_yaw: 0.05
nav_serial_manual_test_pitch: 0.0
```

启动：

```bash
cd ~/sp_vision25_serial_hardened_v5_3
./build-serial/standard_mpc_serial configs/standard3.yaml
```

终端应出现 `MANUAL TEST enabled`，TX 应接近：

```text
tracking=1, yaw_wire=-2.8648, pitch_wire=0.0000, fire=0, fric_on=0
```

电控解析后应看到：

```text
trans_fdb.mode  = 1
trans_fdb.yaw   = -2.8648
trans_fdb.pitch = 0
trans_fdb.shoot = 0
```

看到数据或云台小幅转动后立即按 `Q`/`Esc` 退出，并把 `nav_serial_manual_test` 改回 `false`。

源码推导的默认 yaw scale 是负值。若实际安装方向导致 yaw 转向仍与期望相反，先退出，不要继续闭环；只把 `nav_serial_yaw_scale` 改为 `57.2957795` 后重新做 0.05 rad 单轴测试。pitch 同理，只允许一次改一个轴的符号。

## 6. 正常识别跟随

```bash
cd ~/sp_vision25_serial_hardened_v5_3
./build-serial/standard_mpc_serial configs/standard3.yaml
```

相机窗口会显示亮度、armors 和 targets。确认：

- `enemy_color` 是敌方颜色。
- 装甲板灯条完整、清晰且没有过曝。
- `armors` 先从 0 变成非 0，随后 `targets` 变成非 0。
- `targets=0` 时，TX 必须显示 `tracking=0, yaw_wire=0, pitch_wire=0`。
- `targets>0` 时才允许 `tracking=1`。

## 7. 通信日志判断

正常情况：

```text
[SerialRX] connected=true ... crc8_errors=0, crc16_errors=0 ... fresh_imu=true, fresh_command=true
[SerialTX] frames=... escaped_extra=... errors=0 ... fire=0, fric_on=0
```

- `imu_frames` 和 `frames` 每秒约增加 200。
- `escaped_extra` 会随动态 timestamp 增加，说明下发中的 `0x5A/0x5B` 确实经过转义。
- 拔插 USB 后应看到 disconnected，随后每秒重试，最终再次出现 connected 和新的有效 IMU。
- `fresh_imu=false` 时只发送安全零指令，不发送有效瞄准角。
- `fresh_command=false` 时说明 Planner 超过 200 ms 没有刷新命令，TX 层会独立改发安全零指令。
- 相机或识别链超过 200 ms 没有更新目标时，会出现 `Target input is older`，并立即改发安全零指令。

故障定位：

- RX 正常、TX frames 增长、电控接收计数为 0：核对车上烧录固件、CDC 接收任务和电控观察变量，不能再归因于相机。
- CRC8/CRC16 持续增长：核对车上固件是否与 `newcommunication2.0` 同版，并保持 stuffing=true。
- armors/targets 始终为 0：属于识别或曝光问题，不是串口下发问题。
- 云台只动约期望值的 1/57：实际运行配置仍是 scale=1.0。
- 云台能动但围绕错误基准：先回到 GYRO，让电控刷新 offset，再切 AUTO；仍错误则需要电控明确上传当前模式、冻结 offset 和 `gyro_yaw_inherit`。

注意：电控当前没有 RobotCmd 超时 watchdog。v5.3 能在正常退出、丢目标和 IMU 过期时发安全零帧，但 Linux 断电或进程被强杀后无法继续发包。上车长期运行前应按 `LOWER_CONTROLLER_MIN_CHANGES_V5_3.md` 增加下位机超时保护。
