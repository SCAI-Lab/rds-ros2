# RDS - ROS2

ROS2 port of the Reactive Driving Support system for convex non-holonomic robot navigation.

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/)


- **`ros2-port`** (this branch) — Active ROS2 implementation
- **`crowdbot`/`master`** — Original ROS1 implementation

This is a complete ROS2 reimplementation. For the original ROS1 version, see the [upstream repository](https://github.com/epfl-lasa/rds).

- 📺 [Video demonstration](https://www.youtube.com/watch?v=RAKAhTWd7jw)
- 🔬 Original research funded by EU H2020 [Crowdbot](http://crowdbot.eu/) project

---

## About

**Reactive Driving Support (RDS)** computes safe velocity commands in real time. It takes the robot's desired velocity, its capsule-shaped body, and nearby obstacles (static from LiDAR and dynamic with tracked velocities) and solves an optimisation problem to find the closest safe velocity to the desired command using ORCA (Optimal Reciprocal Collision Avoidance) and velocity obstacles.

The algorithm runs in a closed loop at 10–50 Hz and adapts continuously to changing environments.

---

## Package Structure

```
src/
├── rds_core/          # Core RDS algorithms (ORCA, RVO2, capsule geometry, SDL2 GUI primitives)
├── rds_ros2/          # ROS2 nodes, launch files, config, scripts
├── rds_msgs/          # Custom ROS2 message and service definitions
├── rds_gui_ros2/      # SDL2-based visualisation node (command space + work space)
├── rvo2_lib/          # RVO2 library for velocity obstacles
└── ira_laser_tools/   # Laser scan merging utility
```

---

## Dependencies

| Dependency | Install |
|---|---|
| ROS2 Jazzy | [docs.ros.org](https://docs.ros.org/en/jazzy/Installation.html) |
| SDL2 | `sudo apt install libsdl2-dev` |
| PCL / pcl_conversions | `sudo apt install ros-jazzy-pcl-conversions` |
| tf2 | included with ROS2 desktop |

---

## Building

```bash
cd ~/scai/rds-ros2
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to rds_gui_ros2 rds_ros2
source install/setup.bash
```

To build only the core + ROS2 nodes (no GUI):

```bash
colcon build --packages-up-to rds_ros2
```

---

## Running on the WHILL

Launch the full pipeline:

```bash
ros2 launch rds_ros2 rds_whill.launch.py
```

This starts:
- TF publishers (`base_link` → `main_body_frame` → `rds_frame`)
- LiDAR merger (`ira_laser_tools`) from `/scan_left` and `/scan_right`
- Voxel filter (passthrough + downsampling)
- `rds_node` — core RDS service + obstacle subscriber
- `rds_nominal_command_node` — calls the RDS service and forwards corrected velocity to `/cmd_vel`

### Pedestrian tracking

If a pedestrian tracker is running, make sure it publishes `rds_msgs/msg/PedestrianTracks` to the configured topic. The default topic and parameters are in:

```
src/rds_ros2/config/rds_node_params.yaml
```

```yaml
rds_node:
  ros__parameters:
    enable_pedestrian_tracking: true
    pedestrian_track_topic: "rds/input/pedestrian_tracks"
    default_pedestrian_radius: 0.3   # [m] used when track.radius <= 0
    pedestrian_timeout: 1.0          # [s] drop stale tracks after this
```

Each track must provide: `track_id`, `x`, `y` (position in `rds_frame`), `vx`, `vy` (velocity in `rds_frame`), `radius`.

### GUI (optional)

```bash
ros2 run rds_gui_ros2 rds_gui_node
```

Opens two SDL2 windows:
- **RDS Command Space** — velocity constraints (half-planes), blue = nominal velocity, green = RDS-corrected velocity
- **RDS Work Space** — obstacles around the robot (orange = LiDAR points, cyan = tracked pedestrians with red velocity arrows), green = robot capsule

---

## Test Pipeline

A self-contained test that runs without real hardware:

```bash
ros2 launch rds_ros2 rds_test.launch.py
```

This starts:
- Fake LiDAR (`fake_laser_publisher.py`) — publishes two laser scans with a moving obstacle
- Scan → PointCloud converters + concatenator + voxel filter
- Fake pedestrian publisher (`fake_pedestrian_publisher.py`) — three simulated pedestrians:
  - **Pedestrian 1**: walks straight toward the robot from 4 m ahead (resets at 0.8 m)
  - **Pedestrian 2**: crosses left to right at 2 m ahead
  - **Pedestrian 3**: orbits at 3 m radius continuously
- `rds_node` with pedestrian tracking enabled
- `rds_client_ros_node.py` — calls the RDS service at 10 Hz
- `rds_gui_node` — visualisation windows

### Keyboard teleop

In a second terminal, drive the robot with the keyboard:

```bash
source install/setup.bash
ros2 run rds_ros2 keyboard_teleop.py
```

| Key | Action |
|-----|--------|
| `W` / `S` | Forward / backward (+/- 0.1 m/s) |
| `A` / `D` | Turn left / right (+/- 0.1 rad/s) |
| `Space` | Stop |
| `Q` | Quit |

### What to expect

- **Command space**: ORCA half-plane constraints shift as pedestrians move relative to the robot velocity. Blue arrow = desired velocity, green arrow = RDS output. When an obstacle is in the way, green deviates from blue.
- **Work space**: cyan circles are pedestrians (with red velocity arrows showing predicted position after `tau` seconds). Orange dots are LiDAR points. Green capsule is the robot (always at centre — ego-centric frame).

---

## Configuration

### Robot shape (`rds_client_ros_node.py` or `wrapper_base` config)

Robot geometry and velocity limits are passed per service call. Defaults in `rds_client_ros_node.py`:

| Parameter | Default | Description |
|---|---|---|
| `capsule_radius` | 0.2 m | Robot half-width |
| `capsule_center_front_y` | 0.18 m | Front capsule centre in robot frame |
| `capsule_center_rear_y` | -0.5 m | Rear capsule centre in robot frame |
| `rds_tau` | 1.5 s | Time horizon for velocity obstacles |
| `rds_delta` | 0.1 m | Safety margin added to all obstacle radii |
| `vel_lim_linear_max` | 2.0 m/s | Maximum forward speed |
| `vel_lim_angular_abs_max` | 1.0 rad/s | Maximum angular speed |

For the WHILL, use the values in `config/whill_config_icra`.

### Pedestrian tracking (`config/rds_node_params.yaml`)

See the [Pedestrian tracking](#pedestrian-tracking) section above.

---

## Architecture

```
Teleop / Navigation
        │ cmd_vel_in
        ▼
rds_client_ros_node.py  ──── RDS service call ────►  rds_node
        │                    (10 Hz)                      │
        │ rds_modulated_cmd_vel                           ├── LiDAR points (rds/input/filtered/points)
        ▼                                                 ├── Pedestrian tracks (rds/input/pedestrian_tracks)
    Robot driver                                          └── Publishes rds_to_gui
                                                                    │
                                                               rds_gui_node
```

---

## Topic Reference

| Topic | Type | Direction | Description |
|---|---|---|---|
| `rds/input/filtered/points` | `sensor_msgs/PointCloud2` | → rds_node | Voxel-filtered LiDAR points in `rds_frame` |
| `rds/input/pedestrian_tracks` | `rds_msgs/PedestrianTracks` | → rds_node | Tracked pedestrians with positions and velocities in `rds_frame` |
| `rds_velocity_command_correction` | `rds_msgs/srv/VelocityCommandCorrectionRDS` | service | RDS correction service |
| `rds_to_gui` | `rds_msgs/ToGui` | rds_node → | Full state for visualisation |
| `cmd_vel` | `geometry_msgs/Twist` | rds_client → | Corrected velocity output |



## ToDos

  Connecting a real tracker:
  - callbackPedestrianTracks takes positions directly with no TF transform 
  - Verify the tracker publishes rds_msgs/PedestrianTracks or write a converter node

  Production launch:
  - rds_whill.launch.py doesn't include the GUI node for now



