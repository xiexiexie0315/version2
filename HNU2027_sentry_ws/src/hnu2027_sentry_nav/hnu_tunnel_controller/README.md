# 第一阶段：导航与过洞协调节点

节点只选择速度来源并组合模式请求，不推进HNU过洞状态机，不打开串口，
不模拟机构或宣布动作完成。自瞄目标仍走原通信通路。

## 构建与启动

原工作空间普通安装与symlink缓存混用，本阶段使用独立目录，保留原build/install。
在 /home/robomaster/version2/HNU2027_sentry_ws 下执行：

    source /opt/ros/humble/setup.bash
    colcon --log-base log_stage1 build --build-base build_stage1 --install-base install_stage1 --packages-up-to hnu_tunnel_controller hnu2027_sentry_bringup --symlink-install
    source install_stage1/setup.bash
    ros2 launch hnu2027_sentry_bringup navigation_coordinator.launch.py

仿真加use_sim_time:=true，所有header使用同一ROS时钟。节点内部接收后采用steady clock
看门狗，仿真时钟暂停不会无限维持运动授权。launch只启动协调节点，不启动整车或串口。

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| /navigation/cmd_vel | geometry_msgs/Twist | Nav2输入，需remap现有最终导航速度 |
| /tunnel/cmd_vel | geometry_msgs/TwistStamped | 对齐/穿越控制输出 |
| /tunnel/control_request | hnu_sentry_interfaces/TraverseControl | HNU周期阶段请求 |
| /robot/controller_state | hnu_sentry_interfaces/LowerControllerState | 实际下位机反馈 |
| /robot/control_command | hnu_sentry_interfaces/MotionControlCommand | 唯一协调输出 |
| /robot/emergency_stop | std_msgs/Bool | 软件停止，true保持到显式false，不代替硬件急停 |

速度为base_frame（默认base_link）坐标：x前、y左、z逆时针，m/s和rad/s。
节点不做TF转换；Stamped速度frame_id必须匹配。
Nav2若输出TwistStamped，配置nav2_stamped=true；旧帧、未来帧和错误frame被拒绝。
Twist无源时间戳，按接收时间超时，无法识别上游不断重发的缓存值，上游不能这样保活。
默认速度超时0.25s，请求和反馈超时0.5s，发布50Hz，均可配置。
QoS为volatile、depth=1，不接收持久化历史命令。

## 阶段映射

沿用hnu_tunnel_core的阶段，不在协调器中自动迁移。
后续HNU ROS适配器发布这些请求；本阶段没有接入完整Action服务或自动发布阶段。

| 阶段 | 速度 | 底盘 | 云台 | 姿态 |
|---|---|---|---|---|
| NORMAL/APPROACH | Nav2 | 正常模式 | 自瞄 | 保持 |
| DETECT（准备/归中） | 零 | 导航直接控制 | 归中 | 保持 |
| ALIGN | 过洞速度，需已归中 | 导航直接控制 | 归中保持 | 保持 |
| FOLD | 零 | 导航直接控制 | 收纳 | 停稳且归中才请求折叠 |
| VERIFY | 零 | 导航直接控制 | 收纳 | 保持 |
| TRAVERSE/EXIT | 过洞速度，需实际已折叠 | 导航直接控制 | 收纳 | 保持 |
| UNFOLD | 零 | 导航直接控制 | 收纳 | 停稳才请求展开 |
| COMPLETE | 先零再Nav2 | 恢复正常模式 | 自瞄 | 保持 |
| FALLBACK/ABORT | 零 | 停车 | 收纳要求 | 保持，不自动展开 |

洞口安全、对齐、完全离洞和展开空间由上游HNU负责；阶段名不等于实际安全。
COMPLETE需由上游显式请求，直接回NORMAL不会解除过洞接管。
进入COMPLETE需实际已展开、云台可用且停稳，先输出一次停车再恢复正常模式。
正常模式配置spin/follow/nav_direct。spin和follow由下位机决定wz，
协调器传vx/vy并清零导航wz；nav_direct传完整vx/vy/wz。
Nav2针对小陀螺的路径跟踪适配不在本阶段实现。

## 未来通信适配器契约

消息枚举是ROS契约，不等于STM32当前协议值，接入时必须显式映射。
request_id标识一次意图，重复发送保持编号，意图改变更新编号。
实际底盘模式与accepted_request_id匹配后，才允许非零运动；
发出模式请求本身不等待反馈接受，避免双方互等。

command_valid=false：停车、禁止自转和开火，不发起新的机构动作。
现有收纳状态按下位机机构保持策略处理，不能理解为整车卸力。
command_valid=true且motion_allowed=false：允许模式和机构请求，但底盘禁止运动，
包括小陀螺自转。不能仅看零速度就认为已停止自转。
POSTURE_KEEP不改变姿态目标，GIMBAL_STOW交给下位机收纳控制。
allow_fire需与独立自瞄开火条件相与，它本身不是开火命令。
无有效速度来源时输出STOP，避免导航超时后仍自转。

反馈完成状态必须来自真实机构。通信适配器仅在收到新鲜原始反馈时刷新header，
不能循环发布缓存反馈保持在线，并应处理帧序号、重连和会话。
反馈速度使用同一底盘坐标系。输出命令也需要传输端超时检查。
本阶段未实现串口线路协议，65字节旧RobotCmd尚不能携带新增模式。

## 测试和范围

    source /opt/ros/humble/setup.bash
    source install_stage1/setup.bash
    colcon --log-base log_stage1 test --build-base build_stage1 --install-base install_stage1 --packages-select hnu_tunnel_controller hnu_sentry_interfaces hnu2027_sentry_bringup
    colcon test-result --test-result-base build_stage1 --verbose

核心测试覆盖选择、横移、模式确认、机构请求条件、超时和显式恢复。
ROS测试在隔离域启动真实协调节点并注入测试消息，不连接串口，
不模拟下位机控制算法，也不代表硬件在环或实车验证。

后续仍需HNU阶段发布端、真实通信适配器和下位机实际执行。
