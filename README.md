# Pedestrian Follow Simulation in Python

## Introduction
This project aims to simulate how autonomous vehicles can automatically track pedestrians while maintaining a safe distance.

## Features
- Simulates the behavior of autonomous vehicles in tracking pedestrians.
- Dynamically adjusts the vehicle's speed and direction based on the pedestrian and vehicle's positions.
- Ensures a track distance is maintained between the vehicle and pedestrians.

## Input
- `ped_position`: The relative coordinates of the pedestrian (referenced to the vehicle).
- `vehicle_pose`: The global coordinates, posture, and speed of the vehicle (assumed to be obtainable via IMU, GPS, etc.).

## Simulation Results
The image below demonstrates how the autonomous vehicle tracks a pedestrain and maintains a track distance during the simulation
![Figure_1](https://github.com/JinTTTT/ped_follow_simulation_python/assets/124395755/a616b0af-d9e8-47fe-bd26-74cad632d553)



## Key Points
### Create Pedestrain Coordinate Information
Since the pedestrian coordinate information in actual tests is relative coordinate to the vehicle, so firstly I have define the pedestrian coordinate by globally. Then, using the received infomation of vehicle's global coordinates and yaw angle, i perform a transformation using the coordiante transformation matrix as below:  


#### To transform pedestrian coordinates from the global frame to the vehicle frame:  
![CodeCogsEqn (4)](https://github.com/JinTTTT/ped_follow_simulation_python/assets/124395755/dd5c346c-5fca-46cb-b4d7-9632f97ec0ab)

#### To visualize the transformation of vehicle coordinates back to the global frame:  
![CodeCogsEqn (5)](https://github.com/JinTTTT/ped_follow_simulation_python/assets/124395755/84b63f3f-0e65-40ef-a512-903fd70e7b37)


### Vehicle State Update with Ackermann Model
![CodeCogsEqn (3)](https://github.com/JinTTTT/ped_follow_simulation_python/assets/124395755/6eb9a40d-1b61-486f-b073-b36864a5f260)


### Control
#### Lateral Control
Lateral control is achieved by controlling the vehicle's yaw angle to match the angle from vehicle towards the pedestrian

#### Longitudinal Control
Longitudinal control uses a cascade control strategy: the outer loop controls distance between the vehicle and the pedestrian, and the inner loop controls speed. The logic is to adjust vehicle's speed to match the distance towards the pedestrian, ultimately allowing the vehicle to maintain a set track distance while following the pedestrian.

## Plattform
- ubuntu 22.04
- ros2 humble

## Installation Guide
1. Clone the repository: git clone
2. Install dependencies: rosdep install --from-paths src --ignore-src --rosdistro humble -y
3. Build: colcon build
4. Source the installation: source install/setup.bash
5. Launch: ros2 launch simulation_launch simulation_launch.py



