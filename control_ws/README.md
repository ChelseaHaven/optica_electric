# control_ws

控制工作空间按算法分包，后续算法在 `src` 下建立同级包：

- `control_common`：共用的 ROS 控制接口、目标/里程计解析和周期发包，不包含算法。
- `position_pid`：本次二维位置/航向 PID。`controller.py` 为无 ROS 依赖的数学核心，`node.py` 为消息和状态接口。
- `control_bringup`：控制算法统一启动入口。保留前缀，避免与其他工作空间启动包重名。

各算法采用相同的 ROS 接口，由使用者选择运行哪个控制器：

| 接口 | 类型 | 含义 |
|---|---|---|
| 订阅 `/odom` | nav_msgs/Odometry | odom 坐标系位姿、base_link 坐标系速度 |
| 订阅 `/goal_pose` | geometry_msgs/PoseStamped | odom 坐标系绝对目标 x/y/yaw |
| 发布 `/cmd_vel` | geometry_msgs/Twist | 车体系 vx/vy（m/s）、wz（rad/s） |
| 发布 `/control/status` | std_msgs/String | JSON 状态、目标、误差、到达状态和指令 |

ROS 包名在所有 overlay 工作空间中必须唯一。定位公共包和启动包现为 localization_common / localization_bringup，算法包仍为 eskf。

## 启动

控制工作空间的统一约定：**所有控制算法启动即持续发布 `/cmd_vel`，不设 enable/disable 开关或服务。**
没有目标或里程计无效时发布零速度；收到有效目标和里程计后自动控制。后续算法遵守相同接口。

```bash
source /home/pi/chenzhihan/optica_electric/setup.bash
ros2 launch control_bringup control.launch.py algorithm:=position_pid
```

也可直接 `ros2 launch position_pid position_pid.launch.py`。
设置目标后立即生效，例如 odom 系绝对位置 x=0.2 m、y=0、yaw=0：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
'{header: {frame_id: odom}, pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

目标四元数必须有效；航向 yaw 对应 z=sin(yaw/2)、w=cos(yaw/2)。目标 frame_id 必须为 odom。
目标是 odom 系绝对位置，不是相对当前车体的位移增量。达到目标后保持该位置。
里程计过期 0.2 秒或控制周期超过 0.1 秒时发布零速度；数据恢复后自动继续，不需要任何启用操作。
按 Ctrl+C 退出控制节点停止发布；桥接节点在 0.3 秒指令超时后下发零速度。
下位机手柄仍需处于 HOST 模式并解除 STOP，B 键可以停止电机。
只运行一个控制算法，避免多个节点同时发布 /cmd_vel。

## PID 和配置

配置：`src/position_pid/config/position_pid.yaml`。重启控制节点后生效。

- 50 Hz；在 odom 世界坐标系计算 X/Y 误差和 PID，再旋转到当前车体系下发，支持麦轮横移。
- 航向误差取最短角差，跨 ±pi 不会绕远路。
- D 项使用负的实测速度，避免目标跳变产生微分冲击；新目标清积分但保留上次输出以平滑过渡。
- 平移速度按向量模长限制，角速度单独限制；含速度和加速度限制以及条件积分抗饱和。
- 初始 kp_xy=0.8、kd_xy=0.15、kp_yaw=1.5、kd_yaw=0.1，积分增益默认 0，可按实车调整。
- 默认最大线速度 0.2 m/s、角速度 0.5 rad/s，线加速度 0.3 m/s²、角加速度 0.8 rad/s²。
- 纯平面控制，无路径规划、避障、轨迹跟踪或全局坐标转换；必须使用同一 odom 原点，定位重启前应停止控制节点，再重新确认目标。

调试：`ros2 topic echo /control/status`。算法只依赖标准 ROS 消息，不直接操作串口或电机。

构建和测试：

```bash
cd /home/pi/chenzhihan/optica_electric/control_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
python3 -m pytest -q src/position_pid/test/test_controller.py
ROS_DOMAIN_ID=197 python3 src/position_pid/test/ros_integration.py
```

测试只使用隔离的模拟数据和话题，不发送实车运动指令。

## MPC

新增同级算法包 `mpc`：有限预测时域内求解受约束二次优化，每次只执行第一步速度。
使用二维全向运动学模型，世界系状态 x/y/yaw，优化变量为未来世界系 vx/vy/wz；当前控制量转回 base_link 后发布。
世界系速度是控制输入，不是车辆动力学状态；模型假设下位机能跟踪速度指令，不能代替下位机轮速环调试。

```bash
source /home/pi/chenzhihan/optica_electric/setup.bash
ros2 launch control_bringup control.launch.py algorithm:=mpc
```

与 PID 相同：启动即持续发布 `/cmd_vel`，没有启用服务。没有目标或里程计过期时发零速度，恢复后自动继续。
目标仍为 `/goal_pose`（PoseStamped，odom 系绝对位姿），状态仍为 `/control/status`。
只启动一个控制算法。启动前停止原 PID，切换后重新发送目标。

默认 20 Hz，预测 12 步、每步 0.05 秒，总时域 0.6 秒。最大线速度 0.2 m/s、角速度 0.5 rad/s，最大线加速度 0.3 m/s²、角加速度 0.8 rad/s²。
目标函数包括位置/航向误差、控制量及控制增量，并加大末端状态误差权重。航向采用相对当前位置的最短角差。
线速度和线加速度使用内接八边形线性约束，确保不超出圆形模长上限；角速度/角加速度使用区间约束。
使用 SciPy SLSQP 求解凸二次规划，提供解析梯度、移位热启动和求解可行性检查。SciPy/NumPy 已在该上位机安装，无 OSQP 依赖。
若求解失败，发布零速度并报告 control_error，不偷偷切回 PID。控制状态包含 solver 成功状态、迭代次数、求解耗时和约束残差。

配置：`src/mpc/config/mpc.yaml`；数学核心：`mpc/controller.py`；ROS 封装：`mpc/node.py`。
PID 的用户配置和参数保持原值，ROS 共用接口提取到 control_common。
测试使用隔离话题，不会给实车发送运动指令：
`python3 -m pytest -q src/mpc/test/test_mpc.py` 和 `ROS_DOMAIN_ID=201 python3 src/mpc/test/ros_integration.py`。
