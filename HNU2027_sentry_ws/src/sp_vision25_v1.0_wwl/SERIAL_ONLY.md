# 纯 C++ 串口版

运行入口是 `standard_mpc_serial`，不使用 ROS2。程序直接占用 `/dev/ttyACM0`，所以运行前必须停止导航的串口节点，同一时刻只能有一个程序打开该设备。

完整步骤见 `V5_TEST_GUIDE.md`，通信审计依据见 `COMMUNICATION_AUDIT_V5_3.md`。

关键限制：

- `fire` 和 `fric_on` 始终为 0。
- `nav_serial_byte_stuffing` 必须保持 `true`，与当前 `newcommunication2.0` 一致。
- 自瞄输出弧度，当前电控云台环使用度；yaw 上下行符号相反，因此 yaw scale 为 `-57.2957795`，pitch scale 为 `57.2957795`。
- 电控在 GYRO 模式持续刷新 yaw offset、进入 AUTO 后冻结，上传 yaw 已是相对量，所以 `nav_serial_yaw_relative_to_start` 保持 `false`。
- 正常运行时 `nav_serial_manual_test` 必须为 `false`。
- `nav_serial_command_timeout_ms` 默认 200；Planner 卡住时 TX 层会独立清零。
- 相机窗口按 `Q` 或 `Esc` 正常退出。

```bash
cd ~/sp_vision25_serial_hardened_v5_3
rm -rf build-serial
cmake -S . -B build-serial \
  -DCMAKE_BUILD_TYPE=Release \
  -DSP_VISION_DISABLE_ROS2=ON
cmake --build build-serial --target standard_mpc_serial -j2
./build-serial/standard_mpc_serial configs/standard3.yaml
```
