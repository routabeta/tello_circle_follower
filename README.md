# Tello Circle Tracker (ROS 2 + OpenCV)

This project lets a Tello Boost Combo drone to autonomously follow a circular object using **ROS 2 (Foxy)** and **OpenCV**. It uses **OpenCV** to identify and track circular objects in the drone's onboard camera FOV, and uses **ROS2** to build a framework of nodes that use this information to control the drone along the x, y, and z axes.

---

## Project Overview

ROS 2 and OpenCV are used to build a vision and control system. The system detects circles, tracks them across frames, reinitializes tracking to correct for drift, calculates offsets and relative sizes, and uses a PID controller to adjust the drone’s position accordingly.

The project builds on [clydemcqueen’s `tello_ros`](https://github.com/clydemcqueen/tello_ros.git) driver repo.

---

## Node Overview

| Node Name             | Package         | Description                                                                  |
|------------------------|-----------------|-----------------------------------------------------------------------------|
| `teleop_keyboard_node` | `tello_control` | Manual keyboard control (movement + commands like takeoff/land).            |
| `circle_tracker_node`  | `tello_cv`      | Detects circles in camera frames and runs a tracker on detections.          |
| `pid_controller_node`  | `tello_cv`      | PID controller that centers/resizes the circle publishing on `/cmd_vel`.    |

## Launch Overview

| Launch Name            | Package         | Description                                                                 |
|------------------------|-----------------|-----------------------------------------------------------------------------|
| `teleop_launch.py`     | `tello_driver`  | Launches driver nodes to communicate with Tello.                            |
| `simple_launch.py`     | `tello_gazebo`  | Launches Tello model in Gazebo simulation for testing.                      |
| `circle_tracker.launch.py` | `tello_cv`  | Launches circle tracking/PID nodes.                                         |

---

## Important Parameters

### Circle Detection (in `circle_tracker.launch.py`)
- `target_radius`: Default `60.0`. Target radius in pixels of the target object in the camera feed to govern x-axis movement (distance from circle).
- `scale`: Default `0.75`. Ratio to scale down image before processing to speed up detection/tracking. Smaller = faster but less accurate
- `max_tracker_err`: Default `20`. The error, in pixels, between the center of the tracked vs. detected circle before re-initializing


### PID Controller (in `pid_controller_node`)
- `kp`, `ki`, `kd`: Proportional, integral, derivative gains for control loop. (Currently hardcoded)

---

## Topics

| Topic Name       | Message Type               | Description                                         |
|------------------|----------------------------|-----------------------------------------------------|
| `/cmd_vel`       | `geometry_msgs/msg/Twist`  | Movement commands to the drone.                     |
| `/circle_offset` | `geometry_msgs/msg/Vector3`| x/y offset from image center (z = radius).          |
| `/image_raw`     | `sensor_msgs/msg/Image`    | Raw input from Tello’s onboard camera.              |
| `/image_circled` | `sensor_msgs/msg/Image`    | Processed image with detected circle overlay.       |

---

## Operation Instructions

### 1. Launch the Container
Make sure you are in your workspace + the workspace is built:
```bash
docker compose up
colcon build
```

### 2. Connect to the Tello Drone
Connect to Tello’s Wi-Fi network (`TELLO-XXXXX`) via your external Wi-Fi adapter (should be automatic if `nmcli` is set correctly).

### 3. Start the ROS Nodes
#### Connect to the Drone
```bash
ros2 launch tello_driver teleop_launch.py
```

#### For Simulated Testing (Gazebo)
```bash
ros2 launch tello_gazebo simple_launch.py
```

#### Launch Visualization & Control
```bash
rviz2 -d /root/tello_ros_ws/src/configs/rviz_config.rviz
ros2 run tello_control teleop_keyboard_node
ros2 launch tello_cv circle_tracker.launch.py target_radius:=60 scale:=0.75 max_tracker_err:=20
```

Use keyboard commands:
- **Takeoff**: `t`
- **Land**: `g` or any number key 0–9
- **Manual move**: Use mapped keys from `teleop_keyboard_node`

After takeoff, the drone should follow a circle placed in front of it.

## Networking

The Tello drone hosts its own Wi-Fi network (e.g., `TELLO-435946`). This connection is not super strong out of the box, so I recommend using some sort of Wifi adapter on the desktop/ground station to improve the connection. I used the **NetGear A6210** USB Wifi adapter, and configured the Tello's network to bind to the adapter by default:
```bash
nmcli connection modify TELLO-435946 connection.interface-name wlx80cc9c949553
```
Replace `TELLO-435946` with the network name `wlx80cc9c949553` with the correct interface name used by the adapter. This can be identified with `ip a` or `nmcli device status` or `iw dev`, etc.

---

## Battery

There are two charging options:

- **In-drone Charging:** Use a Micro USB type B cable. Blue light should go from a slow blink (charging) to solid (charged), usually ~20/30 minutes.
- **Multi-Battery Block Charger:** Insert up to 3 batteries, let them charge simultaneously. Yellow means staged for charging, blinking green means charging, solid green means charged, and red indicates a battery error (try pulling it out and replacing it).

---