# TekBot Autonomous Navigation – ROS2 Pathfinding

## Overview

This project was developed during the **Tekbot Robotics Challenge 2025**.

The objective is to implement an autonomous navigation system for a mobile robot in a maze environment using **ROS2 Humble**, **Gazebo** and **Nav2**.
The robot is able to:

* map its environment
* localize itself
* compute a trajectory
* reach a target position while avoiding obstacles

The project reproduces a real robotic navigation pipeline used in autonomous mobile robots.

---

## Technologies & Tools

* ROS2 Humble (Ubuntu 22.04)
* Gazebo Classic
* RViz2
* SLAM Toolbox
* Nav2 (Navigation Stack)
* AMCL Localization
* LIDAR sensors

---

## Navigation Architecture

The navigation system follows the classical robotics pipeline:

1. **Simulation** → Gazebo environment with TekBot robot
2. **Mapping (SLAM)** → building the map using sensor data
3. **Localization (AMCL)** → estimating robot position
4. **Path Planning** → computing optimal path (Dijkstra / NavFnPlanner)
5. **Obstacle Avoidance** → dynamic navigation using Nav2

---

## System Requirements

ROS2 Humble must already be installed.

Install required packages:

```bash
sudo apt update
sudo apt upgrade

sudo apt install ros-humble-teleop-twist-joy
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-rqt*
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

---

## Workspace Setup

Clone the TekBot simulator:

```bash
git clone https://github.com/charif-tekbot/tekbot_sim.git
cd tekbot_sim
source configure.sh
```

Build the workspace:

```bash
colcon build
source install/setup.bash
```

---

## Launch Simulation

Start the robot inside the maze:

```bash
ros2 launch maze_solving tekbot_maze.launch.py
```

Gazebo will open and spawn the robot in the environment.

---

## Mapping (SLAM)

Start SLAM Toolbox:

```bash
ros2 launch slam_toolbox online_async_launch.py
```

Open RViz:

```bash
rviz2
```

Control the robot manually to explore the maze:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

After exploration, save the generated map.

---

## Autonomous Navigation

Localization:

```bash
ros2 launch nav2_bringup localization_launch.py map:=./src/tekbot_sim/tekbot_navigation/maps/tekbot_map.yaml use_sim_time:=true
```

Navigation:

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map_subscriber_transient_local:=true params_file:=./src/tekbot_sim/tekbot_navigation/config/nav2_params.yaml
```

Open RViz:

```bash
rviz2
```

1. Set the robot initial position (**2D Pose Estimate**)
2. Set the target position (**2D Goal Pose**)

The robot autonomously computes a path and navigates to the goal.

---

## Path Planning

The navigation stack uses **NavFnPlanner**, based on the **Dijkstra algorithm**.

The planner:

* receives the robot position from AMCL
* reads the occupancy grid map
* computes the lowest cost path
* sends trajectory to the controller

---

## Project Structure

```
tekbot_navigation/
 ├── launch/
 ├── config/
 ├── maps/
 ├── rviz/
 └── params/
```

---

## Results

The robot successfully:

* maps the maze
* localizes itself
* plans a path
* avoids obstacles
* reaches the goal autonomously

This demonstrates the integration of SLAM and autonomous navigation in a realistic robotics simulation.

---

## Educational Value

This project demonstrates practical skills required in:

* mobile robotics
* autonomous systems
* robotic software integration
* ROS2 navigation stack

---

## Author

**B2MS CleanTech Team**
Tekbot Robotics Challenge 2025
