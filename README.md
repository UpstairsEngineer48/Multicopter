# Hexacopter Waypoint Flight in Ignition Gazebo (ROS 2)

A ROS 2 workspace that flies a simulated six-rotor drone (X4) through 3D waypoints in Ignition Gazebo, with a downward-facing camera for later search-and-rescue perception work. The waypoint follower measures its own path-tracking error.

<!-- Demo video: add link here -->

## What is stock and what is mine

| Part | Source |
|---|---|
| X4 hexacopter model, motor model, velocity-control plugin | Open Robotics (Gazebo Fuel model `X4 UAV Config 1`, Ignition Gazebo example world `multicopter_velocity_control.sdf`) |
| Workspace and package layout, launch file, ROS 2 bridge setup | This repo |
| Waypoint follower with cross-track error logging (`wp.py`) | This repo |
| Camera re-mounted to look straight down (edited model copy) | This repo |
| World reduced to a single drone on a plain ground plane | This repo |

The low-level attitude and motor control is done by Gazebo's `MulticopterVelocityControl` plugin. This repo sends it velocity commands from a position controller.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Ignition Gazebo (Fortress, `ignition-gazebo6`)
- `ros-humble-ros-ign-bridge`
- `colcon`

```bash
sudo apt install ros-humble-ros-ign-bridge python3-colcon-common-extensions
```

## Build

```bash
git clone <this-repo-url> multicopter_ws
cd multicopter_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

Terminal 1, simulator and ROS 2 bridge:

```bash
ros2 launch hexacopter_sim sim.launch.py
```

Terminal 2, waypoint follower:

```bash
ros2 run hexacopter_control wp
```

The drone takes off to 2 m, flies a 3 m square, and the node prints the tracking error when it finishes.

## Packages

```
src/
  hexacopter_description/   X4 model (models/x4_down), camera pitched down
  hexacopter_sim/           world (worlds/hexacopter.sdf) and launch file
  hexacopter_control/       wp.py waypoint follower
```

## How the control works

- The simulator's velocity controller accepts a `Twist` on `/X4/gazebo/command/twist` after `true` is published on `/X4/enable`.
- `wp.py` reads position from `/model/X4/odometry`, computes a velocity toward the current waypoint with a proportional gain, caps its magnitude, rotates it into the body frame, and publishes it at 20 Hz.
- A waypoint counts as reached within 0.2 m.

Settings are constants at the top of `wp.py`:

| Setting | Value |
|---|---|
| `KP` | 1.0 |
| `VMAX` | 1.5 m/s |
| `TOL` | 0.2 m |
| Waypoints | (0,0,2), (3,0,2), (3,3,2), (0,3,2), (0,0,2) |

## Bridged topics

| ROS 2 topic | Direction | Type |
|---|---|---|
| `/X4/gazebo/command/twist` | ROS to Gazebo | `geometry_msgs/Twist` |
| `/X4/enable` | ROS to Gazebo | `std_msgs/Bool` |
| `/model/X4/odometry` | Gazebo to ROS | `nav_msgs/Odometry` |
| `/world/multicopter/model/X4/link/base_link/sensor/camera_front/image` | Gazebo to ROS | `sensor_msgs/Image` |

The camera sensor is still named `camera_front` from the original model, but in this repo it is pitched down 90 degrees. The top of the image is the direction the drone faces.

## Result

Path-tracking error is the distance from the drone to the straight line between consecutive waypoints, sampled at 20 Hz. The initial climb to the first waypoint is excluded.

| Metric | Value |
|---|---|
| Max cross-track error | 0.225 m |
| RMS cross-track error | 0.128 m |
| Samples | 279 |

This is from a single run, using the simulator's ground-truth odometry. Real or estimated position would give larger errors.

## Roadmap

- Own inner-loop controller on the motor-speed topics, with a PID vs LQR comparison against the stock controller
- Human models in the world and a survivor detector on the downward camera
- Yaw control and mapping

## Credits

The X4 model and the example world are from Open Robotics. They are used here under their original licenses.
