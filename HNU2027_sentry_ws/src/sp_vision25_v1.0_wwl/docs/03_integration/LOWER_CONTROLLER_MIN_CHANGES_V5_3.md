# 给电控的最小修改清单

本文件只说明修改要求，本交付没有改动 `D:\embedded`。

## P0：增加 RobotCmd 超时保护

每次 RobotCmd 通过 CRC16 且 LEN/ID 正确后记录 `last_robot_cmd_ms`。控制任务每周期检查：

```c
if (dwt_get_time_ms() - last_robot_cmd_ms > 100) {
    trans_fdb.yaw = 0.0f;
    trans_fdb.pitch = 0.0f;
    trans_fdb.mode = 0.0f;
    trans_fdb.shoot = 0;
    trans_fdb.linear_x = 0.0f;
    trans_fdb.linear_y = 0.0f;
    trans_fdb.angular_z = 0.0f;
}
```

超时值建议先用 100 ms。不要只依赖上位机正常退出，因为 Linux 断电和进程强杀无法发送退出帧。

## P0：固定角度单位合同

推荐协议长期统一为弧度，在电控解析 RobotCmd 时转换：

```c
trans_fdb.yaw = -gm_yaw * RADIAN_COEF;
trans_fdb.pitch = gm_pitch * RADIAN_COEF;
```

其中 `RADIAN_COEF` 必须确认等于 `180 / PI`。yaw 的负号来自当前上行 `offset - INS` 与 PID 反馈 `INS - offset` 的坐标差异。电控一旦这样修改，自瞄配置的两个 scale 必须同时改为 `1.0`；当前 v5.3 已在上位机完成转换和 yaw 反号，未改电控前不要重复处理。

## P0：严格检查 RobotCmd 长度

在固定偏移解析前要求：

```c
id == PP_ID_ROBOT_CMD && len == 43 && total_len == 49
```

当前解析器只做缓冲区上限检查，却按固定偏移读取。错误长度帧可能让旧数据或无效字节进入控制量。

## P1：让 tracking 真正门控云台目标

建议：

```c
if (trans_fdb.mode == 1.0f) {
    // 使用视觉 yaw/pitch
} else {
    // 清零视觉增量或保持电控定义的安全姿态
}
```

现在 mode=0 和 mode=1 的分支都仍会使用收到的 yaw/pitch。v5.3 已在 mode=0 时主动发零，但下位机仍应独立保证安全。

## P1：上传通信诊断

至少提供以下递增计数，串口日志或 debug 帧均可：

- RobotCmd 收到的原始字节数
- 合法 49 字节帧数
- CRC8 错误数
- CRC16 错误数
- 非法转义序列数
- 最近一次合法帧 timestamp

这样能区分“Linux 没发”“CDC 没收”“转义错”“CRC 错”和“控制任务没使用”。

## 自瞄后续需要的数据

基础转向现有 IMU 数据已经够用。为提高精度，建议后续补：

- 实时弹速，单位 m/s，至少 20 Hz；当前自瞄使用固定 22 m/s。
- 当前云台控制模式以及 `gyro_yaw_inherit`/yaw 基准，用于明确绝对角和相对角合同。
- RobotCmd 接收 watchdog 状态和最近合法帧时间。

开火模块本阶段不需要，现有字段可以保留；v5.3 始终发送 fire=0、fric_on=0。
