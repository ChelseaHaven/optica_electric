# localization_ws

ROS 2 Jazzy 二维平面定位工作空间，算法按包并列：

| 包 | 职责 |
|---|---|
| `localization_common` | 公共轮式运动学、传感器安装变换和协方差映射；不包含滤波算法 |
| `eskf` | 本次二维误差状态卡尔曼滤波，纯数学核心与 ROS 节点分开 |
| `localization_bringup` | 统一算法启动入口。后续算法在 src 下另建同级包，并加入启动表 |

输入：`/drive/wheel_states`（标准 JointState，rad/s，按轮名称识别而非数组下标）、`/imu/data_raw`（标准 Imu，rad/s、m/s²）。
轮名：front_left_wheel、front_right_wheel、rear_left_wheel、rear_right_wheel。
输出：`/odom`（nav_msgs/Odometry）、`/localization/status`（JSON 状态）。
各算法统一发布 `/odom` 和 `/localization/status`，由使用者选择运行哪个算法。默认不发布 TF。

## 启动

```bash
cd /home/pi/chenzhihan/optica_electric/localization_ws
source /opt/ros/jazzy/setup.bash
source ../uart_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch localization_bringup estimation.launch.py algorithm:=eskf
```

上电或启动时，车体在水平地面保持静止至少 1 秒，状态从 waiting/calibrating 变为 tracking 后开始发布里程计。
初始化零点就是这次启动时的 x=0、y=0、yaw=0。
串口桥接必须应用新版 `/drive/wheel_states` 及四个标准轮名称配置。若仍运行旧版 wheel_a～wheel_d 消息，需要按 UART 工作空间说明手动重启桥接后再启动定位。`wheel_topic:=...` 仅用于改话题名称，不会重命名轮子。
定位包不发布 cmd_vel，不改变底盘运动模式；不会重启串口桥接。

选作主定位并需要标准 /odom 和 TF 时：

```bash
ros2 launch localization_bringup estimation.launch.py algorithm:=eskf publish_tf:=true odom_topic:=/odom
```

只允许一个定位算法发布 `odom -> base_link`。不输出 map -> odom，也不运行全局定位。
若直接启动算法包：`ros2 launch eskf eskf.launch.py`。
查看：`ros2 topic echo /localization/status` 和 `ros2 topic echo /odom`。

## 配置和坐标

公共配置：`src/localization_common/config/sensors.yaml`；ESKF 噪声和周期：`src/eskf/config/eskf.yaml`。

- base_link：X 前、Y 左、Z 上；IMU 轴向与 base_link 一致，旋转矩阵为单位矩阵。
- IMU 在前两轮中心连线中点，前后轮距 0.160 m，因此相对底盘中心 `(0.080,0,0)` m。
- 轮径 0.077 m、左右轮距 0.196 m、前后轮距 0.160 m，来自 optic_electric2。
- 位置 m，角度 rad；Odometry.pose 在 odom 坐标系，twist 在 base_link 坐标系。
- IMU 角速度/加速度先旋转到 base_link，再补偿 alpha×r 和 omega×(omega×r)。角加速度用一阶平滑差分，时间常数可调。

## 估计内容与限制

标称状态和误差状态都是 8 维：x、y、世界系 vx/vy、yaw、Z 陀螺零偏、车体系 X/Y 加速度零偏。
50 Hz 预测和修正；IMU 预测，四轮正运动学的 vx/vy 修正。麦轮 w 不作为滤波观测。
平移轮速更新包含正确的航向雅可比。状态误差注入后归零；SO(2) 加性航向的重置雅可比为单位阵。
协方差采用 Joseph 更新与连续白噪声离散积分，噪声不会随任务频率改变而错误缩放。

静止初始化估计零偏；运行中持续满足静止条件 0.5 秒后更新零速度和零偏。
默认 stationary_bias_only=true，轮速更新时冻结零偏的增益行，静止观测才估计零偏，避免将一次踢动/轮滑的加速度差异学成永久零偏；如需完整运动中零偏估计，可设置 false 并重新验证噪声参数。
轮速不因创新过大而持续被拒绝；wheel_nis 仅作为诊断输出，打滑造成的观测偏差仍可能进入估计。

当前串口不上传模块姿态作为 Imu.orientation，因此不融合模块 yaw，也不使用未标定磁场。
这是平面模型，假定车体基本水平：仅扣除启动时水平轴静态均值，**没有动态 roll/pitch 重力投影补偿**。
抬车、倾斜、长时间打滑不能作为正常定位条件。没有绝对位置或航向观测，位置和航向仍会漂移。

传感器超过 0.1 秒未更新会报告状态：IMU 超时时不发布新的里程计，轮速超时会报告 imu_only、仅惯性预测。
长任务间隔报告 timing_gap 并跳过该次积分；不会自动把坐标系重置或回退到其他里程计。
异常数值或线性代数错误停止滤波输出并报告 filter_error，需要检查状态后重启节点。
输入没有 MCU 采样时间戳，当前只能使用桥接接收时间，50 Hz 传感器以约 100 Hz 重发不等同于 100 Hz 独立测量。
本节点每 20 ms 只取最新输入一次，轮速每个 stamp 最多更新一次，后续可扩展硬件时间戳。

首次实车校验应比较静止、直行、横移、原地转向和闭环轨迹，核对轮序、方向、杆臂和零偏。
先作为观测定位使用；本工作空间没有接入位置控制。

验证命令：`python3 -m pytest -q src/eskf/test/test_filter.py`；隔离 ROS 集成测试：`ROS_DOMAIN_ID=195 python3 src/eskf/test/ros_integration.py`。

统一话题：所有算法使用 `/odom`（Odometry）和 `/localization/status`（String JSON）；不根据算法名增加话题命名空间。算法包仍独立，由使用者选择启用哪个。

公共和启动包采用 localization_ 前缀以避免其他工作空间包名冲突；算法包仍为 eskf，输出话题仍统一为 /odom 和 /localization/status。

## 纯轮速里程计

新增同级算法包 `wheel_odometry`，只订阅 `/drive/wheel_states`，不订阅 IMU。
四轮 rad/s 经麦轮正运动学得到 vx/vy/wz，使用每周期恒定车体系速度的 SE(2) 精确积分得到平面 x/y/yaw。
轮半径、轮距、轴距和轮名称与公共配置一致。50 Hz 取最新轮速，轮速过期时暂停积分并停止发布新的里程计，恢复后不积分中断期间的数据。
输出仍统一为 `/odom` 和 `/localization/status`，默认不发布 TF，重启该算法即重新建立本地零点。

```bash
source /opt/ros/jazzy/setup.bash
source /home/pi/chenzhihan/optica_electric/localization_ws/install/setup.bash
ros2 launch localization_bringup estimation.launch.py algorithm:=wheel_odometry
```

也可直接 `ros2 launch wheel_odometry wheel_odometry.launch.py`。
与 ESKF 切换前先停止原定位算法；位置控制应停用并在新 odom 原点下重新设置目标。
此算法没有传感器融合或绝对观测，轮滑、轮径/轮距误差会累计为位置和航向漂移。
协方差根据独立轮速测量噪声通过运动学和积分雅可比传播；`wheel_speed_std_rad_s` 默认 0.1 rad/s，属于待标定的噪声参数，不代表能准确覆盖打滑误差。
配置：`src/wheel_odometry/config/wheel_odometry.yaml`。数学核心在 `wheel_odometry/odometry.py`，ROS 接口在 `wheel_odometry/node.py`。
