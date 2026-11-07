# Gazebo上下位机接口适配（第二阶段）

本包连接第一阶段的统一控制命令与真实Gazebo运动/关节反馈。
复用原始外形模型 hnu2027_sentry（不是升降代理模型），原始SDF不修改：
big_yaw、small_yaw、pitch为旋转关节，fold_roll由0转到pi/2进行折叠。

## 快速使用

在远程电脑VNC桌面的终端运行：

    cd /home/robomaster/version2/HNU2027_sentry_ws
    source /opt/ros/humble/setup.bash
    source install_stage1/setup.bash
    ros2 launch hnu_gazebo_adapter coordinator_gazebo.launch.py

这会打开Gazebo并启动协调节点、适配器和ROS/Gazebo桥接。
默认不自行运动；不会启动真实串口、完整Nav2或视觉算法。
在另一个同样source过环境的终端运行可见动作演示：

    ros2 run hnu_gazebo_adapter coordinator_gazebo_demo --ros-args -p use_sim_time:=true

演示：自转并给测试云台目标 → 停转归中 → 直接转向/横移 →
停车折叠 → 折叠保持 → 折叠前进 → 出洞阶段停车 → 展开 →
恢复自转 → 最终停车。
演示输入模拟HNU的阶段要求，但完成判断来自实际Gazebo反馈，不伪造下位机状态。
这里是开阔调试场景；没有自动识别洞口或实际穿过场地洞口。

默认ROS_DOMAIN_ID=181，IGN_PARTITION=hnu_coordination_sim，用于和真实机器人隔离。
查看话题前在终端设置：

    export ROS_DOMAIN_ID=181
    ros2 topic echo /robot/controller_state

无GUI测试加gui:=false。多个独立场景需分别指定simulation_domain_id和ign_partition；
演示程序的--domain-id要与所选ROS域一致。不要在同一仿真域启动真实通信适配器。
若已经有本场景运行，不要重复启动；可以直接运行演示。

## 文件职责

- adapter_core.py：模式解释、动作互锁、测量稳定判断、命令/反馈看门狗。
- adapter_node.py：ROS命令与原生Gazebo控制话题的适配。
- world_builder.py：临时组合原始模型、开阔场景、速度控制、里程计和看门狗插件。
- src/drive_watchdog.cpp：适配器进程退出后，清除Gazebo原生速度控制器保留的速度。
- launch/coordinator_gazebo.launch.py：唯一仿真启动入口，不启动真实串口。
- scripts/coordinator_gazebo_demo：显式执行动作链，并记录实际关节、速度、位置。
- config/gazebo_adapter.yaml：速度/加速度、容差、稳定时间、动作超时等参数。
- test/：接口行为、重复帧、超时、模型保持不变等回归检查。

运行时组合的世界位于/tmp/hnu_coordinator_world_*/coordinator_sim.sdf，
原模型资产保持不变。仍使用上一阶段的build_stage1/install_stage1/log_stage1，
没有再增加一套构建目录。

## 接口

| 话题 | 类型 | 作用 |
|---|---|---|
| /robot/control_command | MotionControlCommand | 协调节点的统一命令输入 |
| /robot/controller_state | LowerControllerState | 基于实际仿真测量的反馈 |
| /model/hnu2027_sentry/cmd_vel | geometry_msgs/Twist | 原生底盘速度控制 |
| /model/hnu2027_sentry/odometry | nav_msgs/Odometry | 实际仿真里程计 |
| /model/hnu2027_sentry/joint_state | sensor_msgs/JointState | 实际关节反馈 |
| /model/hnu2027_sentry/joint/{joint}/cmd_pos | std_msgs/Float64 | 原生关节位置控制 |
| /simulation/gimbal_target | sensor_msgs/JointState | 仅用于仿真的独立云台关节测试目标 |

测试云台目标使用big_yaw、small_yaw、pitch三个关节名，position单位rad，
header.frame_id=base_link，时间戳使用仿真ROS时间。
它不是MPC自瞄对地角接口，不能将真实自瞄消息直接强转成该测试消息。
无测试目标时AUTO保持关节目标，不虚构视觉检测或自动寻敌。

反馈增加execution_state（idle/holding/centering/folding/unfolding/fault），
与accepted_request_id、实际模式和实际完成状态分开。
accepted表示请求已经被接口接受，不表示动作已经完成。

## 动作与完成条件

- 小陀螺转速由适配器配置，带加减速；导航直接控制执行vx/vy/wz。
- motion_allowed=false时，即使请求SPIN，也不会自转。
- 云台归中在停稳后执行，折叠前检查停稳和归中。
- 折叠/展开使用原模型fold_roll目标，动作期间不允许底盘移动。
- 停稳根据实际里程计；归中根据关节实际位置及速度。
- 折叠/展开根据fold_roll实际位置、速度及连续稳定的仿真采样判断。
- 云台运动不会错误地取消fold_roll已经展开的反馈。
- 重复关节/里程计帧不刷新在线时间。仿真暂停后，steady-time看门狗使反馈失效。
- 命令过期停车并保持最近观测姿态，不自动展开。
- 机构长时间不到位会锁存fault，排除问题后需重启适配器。
- 仿真时间回退视为重置故障，重启适配器，避免旧完成状态被继续使用。
- 原生速度看门狗在适配器消失后约0.4s发出零速度，防止最后一条命令永久保持。

## 仿真真实性边界

云台/折叠使用原有Gazebo关节控制器与刚体物理反馈。
底盘使用原生VelocityControl进行理想速度驱动，带适配器限速/加减速；
不是STM32轮速PID、轮地滑移或制动距离的等效验证。
原模型仍沿用初始自碰撞设置，本场景不证明实际折叠包络或洞口净空安全。
没有新增相机图像、雷达点云或真实视觉/完整导航接入。
本阶段验证接口、控制权切换和可见机构动作；下位机实板联调仍需后续完成。

参考原生实现：
https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/velocity_control/VelocityControl.cc
https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/odometry_publisher/OdometryPublisher.cc

## 构建与验证

    source /opt/ros/humble/setup.bash
    colcon --log-base log_stage1 build --build-base build_stage1 --install-base install_stage1 --packages-up-to hnu_gazebo_adapter --symlink-install
    source install_stage1/setup.bash
    colcon --log-base log_stage1 test --build-base build_stage1 --install-base install_stage1 --packages-select hnu_gazebo_adapter hnu_tunnel_controller hnu_tunnel_core hnu2027_sentry_description hnu2027_sentry_simulation hnu2027_sentry_bringup hnu_sentry_interfaces
    colcon test-result --test-result-base build_stage1 --verbose

完整演示报告默认写到/tmp/hnu_stage2_demo_report.json，可通过--report指定路径。
现场开发验证还记录了/tmp/hnu_stage2_fault_report.json：
暂停仿真会撤销反馈有效性；取消后恢复仿真保持停车；
强制结束适配器后，原生看门狗使实际底盘速度归零。
这些验证不等同于真实硬件或自动过洞的验收。
