# Pedestrian Follow Simulation in Python

## 简介
这个项目旨在模拟自动驾驶汽车如何在保持安全距离的同时自动跟踪行人。

## 功能
- 模拟自动驾驶汽车跟踪行人的行为。
- 根据行人和车辆的位置动态调整车辆的速度和方向。
- 确保车辆与行人之间保持安全距离。

## 输入
- `ped_position`: 行人的相对坐标（以车辆为参考系）。
- `vehicle_pose`: 车辆的绝对坐标、姿态和速度（假设可以通过IMU、GPS等获得）。

## 运行结果
下图展示了模拟过程中，自动驾驶汽车如何跟踪行人并保持安全距离。
![Simulation Result](https://github.com/JinTTTT/ped_follow_simulation_python/assets/124395755/a438e45c-ad5b-445b-b541-f3851ecf7404)

## 关键技术
### 模拟行人坐标信息
因为实际测试中，能够获取到的行人坐标信息是车辆坐标系下的坐标。因此在模拟行人坐标时首先按照全局坐标来定义，然后通过收到的车辆坐标和横摆角信息，使用坐标转换矩阵进行转换。

To transform pedestrian coordinates from the global frame to the vehicle frame:

| ped_rel_x |   | cos(yaw)  sin(yaw) | | translated_ped_x |
| ped_rel_y | = | -sin(yaw) cos(yaw) | | translated_ped_y |

To visualize the transformation of vehicle coordinates back to the global frame:
| global_ped_x |   | cos(yaw) -sin(yaw) | | msg.x |
| global_ped_y | = | sin(yaw)  cos(yaw) | | msg.y |



### Vehicle State Update with Ackermann Model

The new state of the vehicle is calculated as follows:

- Velocity: \( v = v_{new} \)
- Position X: \( x = x + v \cdot \cos(yaw) \cdot dt \)
- Position Y: \( y = y + v \cdot \sin(yaw) \cdot dt \)
- Yaw: \( yaw = yaw + \frac{v}{L} \cdot \tan(steering) \cdot dt \)

where:
- \( v \) is the vehicle's velocity
- \( x, y \) are the vehicle's current positions
- \( yaw \) is the vehicle's heading direction
- \( L \) is the wheelbase of the vehicle
- \( steering \) is the current steering angle
- \( dt \) is the time step


### 控制
#### 横向控制
横向控制使用了横摆角控制方式：即控制车辆自身横摆角，使其等于车辆朝向行人的横摆角。

#### 纵向控制
纵向控制使用了cascade控制：cascade外环为距离控制， 内环为速度控制。控制逻辑是车辆自动调节自身速度匹配与前方目标间距离，最终使得车辆一直能够保持设定的距离来跟随行人。

## 版本信息
1. Ubutu 22.04
2. ros2 humble

## 安装指南
1. 克隆仓库：gitclone
2. 安装依赖：rosdep install --from-paths src --ignore-src --rosdistro humble -y
3. 编译: colcon build
4. source install/setup.bash
5. 启动：ros2 launch simulation_launch simulation_launch.py



