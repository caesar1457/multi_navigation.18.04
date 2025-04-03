# 🤖 Multi-Robot Navigation with Task Allocation (ROS1)

This ROS1-based project enables **multi-robot navigation** and **task allocation** in a simulated environment using TurtleBot3. The system supports namespace-based robot spawning, multi-goal assignment, and collision-free path planning via `move_base`.

> ✅ Independently developed by Caesar Zhao (MSc Robotics, UTS)

---

## 🚩 Project Overview

- **Multi-robot setup** with separate namespaces (`tb3_0`, `tb3_1`, ...)
- **Map-based navigation** using AMCL and `move_base`
- **Task allocation** using custom messages and goal dispatching
- **Simulated environment** with custom Gazebo world

---

## 🚀 Quick Start

### 🧭 Launching Multiple Robots

```bash
ROS_NAMESPACE=tb3_0 roslaunch turtlebot3_bringup turtlebot3_robot.launch
ROS_NAMESPACE=tb3_1 roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

### 🌍 Launch Full Navigation Stack

```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch multi_navigation multi_navigation.launch
```

### 🎯 Assign Multi-Goal Tasks

```bash
rostopic pub /task_allocation std_msgs/String '{data: "{\"tb3_0\": [19,21,27,19], \"tb3_1\": [22,26,28,21]}"}'
```

---

## 📁 Notable Files & Structure

| Folder/File | Purpose |
|-------------|---------|
| `config/*.yaml` | Costmap, planner, and AMCL parameter tuning |
| `launch/*.launch` | Launch files for robots, navigation stack, and task planner |
| `maps/` | Custom lab map (`lab_maps.pgm`, `map.yaml`) |
| `msg/` | Custom ROS message types for matrix and task encoding |
| `rviz/` | RViz configurations for planning and monitoring |
| `src/path_planner.py` | Core path planner and task allocator logic |
| `src/targets.py` | Target parsing and dispatch |
| `urdf/*.xacro` | TurtleBot3 URDFs (with simulation and planning plugins) |
| `world/` | Custom Gazebo world definitions |

---

## 🧠 Technical Highlights

- **Namespace Isolation**: Supports any number of robots with separate TF trees and topics.
- **Task Allocation**: Dynamically dispatches goal sequences to each robot via JSON.
- **Global/Local Planning**: Uses `move_base`, DWA planner, and costmap-based navigation.
- **Custom Messages**: Matrix and vector messages for flexible task representation.

---

## 📜 License

This project is provided for academic and demonstration purposes only.  
© 2024 Caesar Zhao. All rights reserved.

---

## 📬 Contact

**Zhiye (Caesar) Zhao**  
📧 zhiye.zhao-1@student.uts.edu.au  
🌐 [Portfolio Website](https://caesar1457.github.io/zhiyezhao/)
