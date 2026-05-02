# ROS2 Drone Control and Nav2 Autonomous Navigation System

## 📌 Overview

This project implements a **complete robotics system in ROS2**, combining:

* 🚁 **Drone control in simulation (Gazebo)**
* 🤖 **Autonomous navigation using Nav2 (TurtleBot simulation)**

The system demonstrates both:

* **Low-level motion control** (velocity-based drone control)
* **High-level autonomy** (goal-based navigation)

It was developed as part of a robotics coursework and showcases real-world robotics concepts such as control systems, simulation integration, and autonomous navigation.

---

## 🧠 System Architecture

The project is built using a **multi-package ROS2 workspace**, where each package handles a specific responsibility.

```plaintext
src/
├── gamma_droid_bridge_pkg/
├── gamma_droid_controller_pkg/
├── gamma_droid_navigation_pkg/
└── sfr_coursework2_interface_package/
```

---

# 🚁 Section 1: Drone Control System (Gazebo)

## 🔍 Description

A simulated drone (“flying box”) is controlled inside Gazebo using velocity commands.

The system interacts with the simulation using ROS2 topics and transforms, enabling real-time motion control.

---

## ⚙️ Packages Involved

### 🔹 `gamma_droid_bridge_pkg`

* Bridges communication between **Gazebo and ROS2**
* Uses `ros_gz_bridge`
* Handles:

  * `/model/box/cmd_vel`
  * `/tf` transformations

---

### 🔹 `gamma_droid_controller_pkg`

* Implements the **control logic**
* Uses:

  * `tf2` → to track drone position
  * `Twist` → to send velocity commands

---

### 🔹 `sfr_coursework2_interface_package`

* Defines custom ROS2 interfaces:

  * Action messages
  * Communication structures

---

## 🎯 Control Strategy

The drone is controlled using an **Action Server**:

```
robot/set_pose
```

### Behaviour:

* Reads current pose using `tf2`
* Computes velocity commands
* Moves toward target position
* Stops when:

  * ✔ Within **0.1 m** → success
  * ❌ After **5 seconds** → failure
* Provides continuous feedback during execution

---

## 🔑 Key Concepts Demonstrated

* ROS2 Action Server implementation
* Real-time feedback loops
* Velocity-based control
* Simulation integration (Gazebo)
* Transform handling using `tf2`

---

# 🤖 Section 2: Autonomous Navigation System (Nav2)

## 🔍 Description

A TurtleBot simulation is used with the ROS2 Nav2 stack to perform **autonomous navigation in a mapped environment**.

This part focuses on high-level decision-making rather than low-level control.

---

## ⚙️ Package Involved

### 🔹 `gamma_droid_navigation_pkg`

* Interfaces with Nav2
* Handles:

  * Initial pose setup
  * Goal sending
  * Navigation monitoring

---

## 🎯 Navigation Behaviour

The system performs the following steps:

1. Waits for Nav2 stack to be ready
2. Publishes initial robot pose
3. Sends a navigation goal
4. Monitors execution
5. Completes when:

   * ✔ Goal reached within **0.5 m tolerance**
   * ✔ Completed within **60 seconds**

---

## 🔑 Key Concepts Demonstrated

* ROS2 Nav2 stack usage
* Goal-based navigation
* Autonomous motion planning
* Integration with simulation environments

---

# 🔄 Combined System Insight

This project demonstrates a **full robotics pipeline**:

* Low-level control (drone motion)
* High-level autonomy (navigation)
* Simulation-based testing
* Modular ROS2 architecture

---

# ▶️ How to Run

## 1. Source ROS2

```bash
source /opt/ros/jazzy/setup.bash
```

---

## 2. Build workspace

```bash
colcon build
source install/setup.bash
```

---

## 🚁 Run Drone Control System

### Launch bridge

```bash
ros2 launch gamma_droid_bridge_pkg bridge_launch.py
```

### Run controller

```bash
ros2 run gamma_droid_controller_pkg controller_node
```

---

## 🤖 Run Navigation System

### Launch Nav2 simulation

```bash
ros2 launch nav2_bringup tb4_simulation_launch.py
```

### Run navigation node

```bash
ros2 run gamma_droid_navigation_pkg navigation_node
```

---

# 📊 Expected Results

* Drone moves to a specified pose using velocity control
* Action server provides feedback and result
* TurtleBot navigates autonomously to a goal location
* Both systems run reliably within defined constraints

---

# 📚 Learning Outcomes

This project demonstrates:

* ROS2 architecture (multi-package design)
* Action Server implementation
* Gazebo simulation integration
* Autonomous navigation using Nav2
* Real-time robotics system design

---

# 🚀 Future Improvements

* Add PID control for smoother drone motion
* Integrate obstacle avoidance for drone
* Visualize navigation paths
* Combine drone and navigation into a unified system
* Extend to real-world robot hardware

---

# 🏫 Academic Context

Developed as part of:
**Software for Robotics – Coursework 2**

---

# 👤 Author

Arun Kumar
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
