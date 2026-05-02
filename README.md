# ROS2 Drone Control and Nav2 Autonomous Navigation System

## 📌 Overview

This project implements a **complete ROS2-based robotic system** combining:

* 🚁 Drone control using Gazebo simulation
* 🤖 Autonomous navigation using Nav2 (TurtleBot simulation)

It demonstrates both **low-level motion control** and **high-level autonomous navigation** within a single ROS2 workspace.

---

# 🚁 Section 1: Drone Control System

## Description

A simulated drone (“flying box”) is controlled in Gazebo using velocity commands.

## Packages Involved

* `gamma_droid_bridge_pkg` → connects Gazebo and ROS2
* `gamma_droid_controller_pkg` → controls drone motion
* `sfr_coursework2_interface_package` → action/message definitions

## Features

* ROS2 ↔ Gazebo communication (`ros_gz_bridge`)
* Velocity control using `Twist`
* Action Server: `robot/set_pose`
* Real-time pose tracking using `tf2`

## Behaviour

* Moves drone to a target pose
* Success when within **0.1 m**
* Fails after **5 seconds timeout**
* Provides continuous feedback

---

# 🤖 Section 2: Autonomous Navigation (Nav2)

## Description

A TurtleBot simulation is used with the ROS2 Nav2 stack for autonomous navigation.

## Package Involved

* `gamma_droid_navigation_pkg`

## Features

* Initial pose publishing
* Goal-based navigation
* Autonomous movement using Nav2

## Behaviour

* Waits for Nav2 system
* Sets initial pose
* Sends navigation goal
* Reaches target within **0.5 m tolerance**
* Completes within **60 seconds**

---

# 🧠 Key Concepts

* ROS2 Topics, Services, and Actions
* tf2 Transform system
* Gazebo simulation integration
* Velocity-based robot control
* Autonomous navigation (Nav2)

---

# 📂 Project Structure

```
src/
├── gamma_droid_bridge_pkg/
├── gamma_droid_controller_pkg/
├── gamma_droid_navigation_pkg/
└── sfr_coursework2_interface_package/
```

---

# ▶️ How to Run

## 1. Source ROS2

```
source /opt/ros/jazzy/setup.bash
```

---

## 2. Build workspace

```
colcon build
source install/setup.bash
```

---

## 3. Run Drone System

### Launch bridge

```
ros2 launch gamma_droid_bridge_pkg bridge_launch.py
```

### Run controller

```
ros2 run gamma_droid_controller_pkg controller_node
```

---

## 4. Run Navigation System

### Launch Nav2 simulation

```
ros2 launch nav2_bringup tb4_simulation_launch.py
```

### Run navigation node

```
ros2 run gamma_droid_navigation_pkg navigation_node
```

---

# 📚 Learning Outcomes

* ROS2 system architecture
* Action Server implementation
* Simulation-based robotics development
* Autonomous navigation using Nav2
* Real-time robot control

---

# 🏫 Academic Context

Software for Robotics – Coursework 2

---

# 👤 Author

Ansper Miranda
