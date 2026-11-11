# 第三阶段：Nav2 与 HNU 自主过洞闭环

本包把普通导航、已有 HNU 感知/状态机、第一阶段协调器和第二阶段 Gazebo 下位机适配器接起来。
只提交洞后终点，机器人自行到洞前、退出小陀螺、归中对齐、折叠、穿越、展开归正，再恢复普通导航到终点。
自瞄算法仍走独立通路；这里验证云台模式交接和真实关节响应，不模拟目标识别或射击。

## 启动

在远程电脑 /home/robomaster/version2/HNU2027_sentry_ws 执行：

    source /opt/ros/humble/setup.bash
    colcon --log-base log_stage1 build --build-base build_stage1 --install-base install_stage1 --packages-up-to hnu2027_sentry_autonomy --symlink-install
    source install_stage1/setup.bash
    ros2 launch hnu2027_sentry_autonomy autonomous_tunnel.launch.py gui:=true use_rviz:=true

使用本机 VNC 桌面启动即可。SSH 启动图形窗口时使用已有桌面的 DISPLAY=:0、XAUTHORITY=/run/user/1000/gdm/Xauthority 和 XDG_RUNTIME_DIR=/run/user/1000，不需要 X 转发。

另开终端，source 同一环境后：

    ros2 run hnu2027_sentry_autonomy submit_tunnel_goal --domain-id 182 --x -1.2 --y 3.5

也可在本 launch 的 RViz 使用 2D Goal Pose，目标发到 /mission/goal。默认 ROS 域 182、IGN 分区 hnu_autonomy_sim；可通过 simulation_domain_id 和 ign_partition 改为隔离实例。
查看状态：

    ROS_DOMAIN_ID=182 ros2 topic echo /mission/status

当前只支持蓝方出生点通道的正 X 穿越：出生点 (-4,3.5)，洞前点 (-3.2,3.5)，洞中心 (-2.5,3.5)，洞后点约 (-1.8,3.559)。
目标应位于洞后，默认 (-1.2,3.5)。已在洞后时会拒绝再次从入口执行同一路线；重新演示请重启场景。
尚未实现任意洞口自动选路、红方或反向过洞。

## 信息流与完成条件

- Gazebo 原始里程计、关节和真实 GPU 激光 -> simulation_sensors -> TF、/scan、/tunnel/cloud。
- /scan + 原项目地图 -> AMCL；AMCL map->odom + 测量 odom->base_footprint -> 机器人地图位姿。
- Nav2（原有全向 PID 路径跟踪插件）-> /navigation/cmd_vel。
- HNU 点云检测 -> /tunnel/observation；HNU 状态机由 tunnel_mission_node 调用。
- mission -> /tunnel/control_request、/tunnel/cmd_vel -> navigation_coordinator。
- 唯一 /robot/control_command -> Gazebo 下位机适配器 -> 底盘/关节执行。
- 实际 /robot/controller_state（请求编号、模式、速度、归中和折叠状态） -> 协调器与 mission。

NORMAL/APPROACH 使用 Nav2；DETECT 停止小陀螺并请求云台归中；ALIGN 先旋转对齐再横移；FOLD 等实际停稳/归中后折叠。
折叠前再次要求当前安全观测，避免对齐期间出现的障碍被遗漏。VERIFY 复核新鲜障碍观测和实际折叠；TRAVERSE 使用过洞速度；完全离开洞口才 UNFOLD。
新增 RESTORE 阶段在展开后归回普通行驶中心线并停稳，避免偏置折叠轨迹直接恢复旋转导致碰墙。
COMPLETE 显式释放接管，清除动态代价地图并等待新扫描更新，然后重新调用 Nav2 到最终目标。到达后停止。
阶段转换依赖测量与反馈，不通过固定等待时间宣布动作成功。

停止、自转、底盘直接控制、云台归中/收纳、折叠/展开均经现有结构化消息；没有新增串口字节协议。
实车可复用 mission/协调器/HNU 逻辑，以真实定位/传感器和真实通信适配器替换仿真部分；下位机必须实现同一反馈契约。

## 仿真的测量边界

- AMCL 启动时仅提供一次已知出生位置初值；之后用实际激光定位，不连续注入 Gazebo 真值作为定位。
- base_footprint 表示平面导航；base_link 保留测量的高度和姿态，激光/云台 TF 随实际关节变化。
- 点云历史以 1cm 体素保留实际量到的洞口几何；障碍检查使用当前帧。旧几何不代表当前无障碍，当前数据稀疏/丢失不能假定通行。
- 穿越中观测短暂丢失立即禁止运动，连续丢失超时中止；新鲜的危险观测直接中止。
- 使用原始机器人旋转折叠关节和碰撞几何。展开旋转包络半径约 0.374m；折叠高度约 0.223m、宽约 0.646m，折叠后左右不对称，所以过洞中心线偏移约 0.059m。
- 原场地 CAD 显示网格与物理碰撞网格存在差异，且导出场景缺少出生区域地板。本包生成临时场景，补充物理地板，不修改原场景/机器人文件。
- 默认 lidar_geometry:=collision：激光读取原物理碰撞几何的可见代理，使传感器和物理障碍一致；GUI 可能出现灰色代理覆盖。lidar_geometry:=visual 使用原 CAD 外观网格，可能因额外可见横杆拒绝通行。当前通过测试代表碰撞几何基线，不能据此宣称 CAD 外观和实地完全一致。
- 底盘仍是理想速度执行模型，未验证真实轮胎打滑、电机 PID、惯性、小陀螺扰动或串口延时；这些需要后续硬件在环/实车标定。

## 自动验收

    ros2 run hnu2027_sentry_autonomy validate_autonomy

在独立 ROS 域/IGN 分区启动两次完整场景，提交最终目标，不直接注入阶段：
1. 正常路线：检查所有过洞阶段、恢复小陀螺、最终停止展开，以及 Gazebo 原始位置到目标误差 < 0.15m。
2. 洞内实体障碍：检查不进入 TRAVERSE，停在洞外并中止，原始速度近零且保持展开。

每次日志和 JSON 报告写入独立 /tmp/hnu_autonomy_validation_*；程序仅清理自己启动的进程。
可加 --gui 查看验收场景。人为中断或失败应查看 summary.json 和对应 runtime/goal 日志。

    colcon --log-base log_stage1 test --build-base build_stage1 --install-base install_stage1 --packages-select hnu_tunnel_core hnu_tunnel_controller hnu_tunnel_perception hnu2027_sentry_autonomy hnu_gazebo_adapter hnu2027_sentry_description hnu2027_sentry_simulation hnu2027_sentry_bringup hnu_sentry_interfaces
    colcon test-result --test-result-base build_stage1

单元测试覆盖物理几何、传感器坐标、自体过滤、感知有效性和当前障碍检查、状态机恢复/超时、协调器模式与反馈门控。

## 本轮验收记录（2026-11-07）

11 个依赖包构建成功；相关测试 95 项通过，0 失败。
自动双场景验收报告：/tmp/hnu_autonomy_validation_6r0yllga/summary.json。
正常路线原始 Gazebo 终点 (-1.23352,3.50852)，距目标 0.03459m，速度为零、折叠关节约 0。
障碍路线在折叠前以 unsafe_before_fold 中止，实际位置 (-3.23024,3.58059)，速度为零、保持展开、未进入穿越。
上述结果限定于本 README 说明的碰撞几何和理想底盘仿真，不代表实车完成验收。
