# optica_electric

二维麦轮机器人 ROS 2 上位机工程，使用 ROS 2 Jazzy 和 Python。通过串口接收下位机轮速、IMU 数据并下发车体系速度，定位与控制算法分包维护。

## 工程结构

| 工作空间 | 内容 |
|---|---|
| `uart_ws` | `serial_bridge`：串口协议、传感器上报与速度下发 |
| `localization_ws` | `localization_common`、`localization_bringup`，以及并列的 `eskf`、`wheel_odometry` 算法 |
| `control_ws` | `control_common`、`control_bringup`，以及并列的 `position_pid`、`mpc` 算法 |

包内部保留标准 ROS 2 Python 包目录结构。下位机固件不包含在本仓库中。

## 构建

安装 ROS 2 Jazzy、colcon、rosdep 及 Python NumPy/SciPy 后，在仓库根目录依次构建：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths uart_ws/src localization_ws/src control_ws/src --ignore-src -r -y
(cd uart_ws && colcon build --symlink-install)
source uart_ws/install/local_setup.bash
(cd localization_ws && colcon build --symlink-install)
source localization_ws/install/local_setup.bash
(cd control_ws && colcon build --symlink-install)
source ./setup.bash
```

`setup.bash` 只加载环境，不启动节点。每个新终端均需 source 仓库根目录下的此脚本。

## 启动

以下命令分别在三个已加载环境的终端运行：

```bash
ros2 launch serial_bridge serial_bridge.launch.py
ros2 launch localization_bringup estimation.launch.py algorithm:=eskf
ros2 launch control_bringup control.launch.py algorithm:=position_pid
```

定位可选择 `wheel_odometry`，控制可选择 `mpc`。同一时间只运行一个定位算法和一个控制算法。
控制节点启动即周期发布速度；没有有效目标或里程计过期时发布零速度，数据恢复后自动继续。

## ROS 接口

| 话题 | 消息 | 含义 |
|---|---|---|
| `/drive/wheel_states` | `sensor_msgs/msg/JointState` | 四轮角速度，rad/s |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 原始角速度 rad/s、加速度 m/s² |
| `/imu/magnetic_field_raw` | `std_msgs/msg/Float64MultiArray` | 磁力计原始读数 |
| `/odom` | `nav_msgs/msg/Odometry` | 定位输出，odom / base_link |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | odom 系绝对目标位姿 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 车体系 vx/vy，m/s；wz，rad/s |
| `/serial/status`、`/localization/status`、`/control/status` | `std_msgs/msg/String` | 各模块 JSON 诊断状态 |

例如发送 x=0.2 m、y=0、yaw=0 的绝对目标：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
'{header: {frame_id: odom}, pose: {position: {x: 0.2, y: 0.0}, orientation: {w: 1.0}}}'
```

yaw 转四元数：z=sin(yaw/2)、w=cos(yaw/2)，角度计算使用 rad。定位原点改变后需要重新确认目标。

## 配置与说明

串口、传感器安装关系、噪声和控制参数均保存在各包 `config` 目录。提交的参数保留当前实车配置，并非通用调参值。
IMU 位于前轮中心连线中点，方向与 base_link 一致。ESKF 按水平二维运动建模，无绝对定位观测时仍会漂移。
更多协议、参数及测试说明见各工作空间 README。

编译产物、缓存、运行日志与本地密钥不纳入版本管理。
