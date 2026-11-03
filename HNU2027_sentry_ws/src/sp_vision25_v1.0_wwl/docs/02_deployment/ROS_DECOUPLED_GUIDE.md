# ROS 2 解耦运行指南

此工程采用“通信网关独占串口、自瞄只使用 ROS 话题”的运行方式。不要启动任何自瞄侧直连串口程序。

## 组件与职责

- `standard_robot_pp_ros2`：唯一打开下位机串口，发布 `serial/imu` 和 `serial/gimbal_joint_state`，接收最终的底盘与云台命令。
- `gimbal_manager`：将 `cmd_gimbal` 转换为 `cmd_gimbal_joint`。
- `standard_mpc_ros2`：运行相机、自瞄检测、跟踪和 MPC；经 `RosGimbal` 订阅状态并发布 `cmd_gimbal`。
- 导航栈：发布 `cmd_vel`。

## 启动顺序

1. 先启动 `standard_robot_pp_ros2` 的 launch 文件，使其成为唯一串口拥有者。
2. 启动导航栈，确认其持续发布 `cmd_vel`。
3. 启动 `standard_mpc_ros2 configs/standard3.yaml`。

禁止同时运行旧的 `standard_mpc_serial`、`standard_mpc_serial_nav_bridge` 或任何直接打开下位机串口的自瞄程序。
