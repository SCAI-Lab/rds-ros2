# RDS - ROS2

ROS2 port of the Reactive Driving Support system for convex non-holonomic robot navigation

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/)

## ⚠️ Branch Information

- **`ros2-port`** (this branch) - Active ROS2 implementation  
- **`crowdbot`/`master`** - Original ROS1 implementation from LASA

This is a complete ROS2 reimplementation. For the original ROS1 version, see the [upstream repository](https://github.com/epfl-lasa/rds).

---

## About

**Reactive Driving Support (RDS)** is a method for robots to avoid imminent collisions with moving obstacles, developed at LASA, EPFL. 

- 📺 [Video demonstration](https://www.youtube.com/watch?v=RAKAhTWd7jw)
- 🔬 Original research funded by EU H2020 [Crowdbot](http://crowdbot.eu/) project

This ROS2 port maintains the core reactive navigation functionality while adapting to modern ROS2 architecture and integrating with the WHILL platform.

### Key Features

- ✅ **Reactive collision avoidance** for non-holonomic capsule-shaped robots
- ✅ **ROS2 native implementation** with modern APIs
- ✅ **Laser scanner integration** (front/rear LiDAR support)
- ✅ **WHILL platform integration**
- ✅ **Service-based architecture** for velocity command correction
- ✅ **RVO2 library integration** for velocity obstacles

---

## Package Structure

```bash
src/
├── rds_core/          # Core RDS algorithms and logic
├── rds_ros2/          # ROS2 node implementations and launch files
├── rds_msgs/          # Custom ROS2 message/service def
├── rvo2_lib/          # RVO2 library for velocity obstacles
└── ira_laser_tools/   # Laser scan processing utils
```