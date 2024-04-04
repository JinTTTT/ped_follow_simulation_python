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

## 技术细节
- **模拟环境**：使用Ackermann模型来模拟汽车的运动。
- **输入处理**：处理和解析行人数据(`AVL_Example.json`)，提取行人和车辆的位置信息。
- **控制策略**：包括横向控制（减少车辆与行人的横向距离）和纵向控制（控制车辆速度，保持与行人的安全距离）。

## 安装指南
1. 克隆仓库： gitclone
2. 安装依赖，编译
3. source后通过ros2 launch启动：ros2 launch simulation_launch simulation_launch.py 
