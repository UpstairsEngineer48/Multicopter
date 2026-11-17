# hexacopter_control

Autonomous search-and-map mission for an X4 hexacopter in Gazebo (ROS 2, Python).
The drone sweeps an arena with a downward camera, builds an occupancy map from the images,
finds coloured victim markers, plans a tour through them with A*, and flies that tour.

## Demo

[![Autonomous hexacopter mapping and victim search demo](https://img.youtube.com/vi/8iAXApqemwE/maxresdefault.jpg)](https://www.youtube.com/watch?v=8iAXApqemwE)

Click the thumbnail to watch the full mission: sweep, map building, victim detection, A* planning and path following.

## What it does

1. **Sweep** (`wp`): flies a lawnmower pattern at 2 m so the camera footprint covers the whole arena.
2. **Map** (`mapper`): turns every overhead frame into obstacle/free votes on a world-frame grid.
3. **Detect** (`victim_detector`): finds red, green and blue marker plates and locates them in the world.
4. **Plan** (`planner`): when the sweep ends, runs A* from the start through every victim and back home.
5. **Follow** (`follower`): flies the planned path with a pure-pursuit controller and hovers at each victim.

The map is saved to `~/hexacopter_map.pgm` and `~/hexacopter_map.yaml`, and the victims to `~/victims.json`.

## Quick start

```bash
cd ~/multicopter_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
ros2 launch hexacopter_sim full_mission.launch.py
```

Options:

```bash
ros2 launch hexacopter_sim full_mission.launch.py rviz:=false
ros2 launch hexacopter_sim full_mission.launch.py wp_delay:=20.0
```

`wp` starts the sweep, so it is delayed by `wp_delay` seconds (default 10) to let Gazebo load and the other nodes subscribe first.
`hexacopter_control/launch/mission.launch.py` runs the same nodes without a delay and can also skip the simulator (`sim:=false`).
To plan again without restarting: `ros2 topic pub --once /replan std_msgs/msg/Empty "{}"`.

## Packages

| Package | Contents |
|---|---|
| `hexacopter_description` | X4 model with a downward camera (`models/x4_down`) |
| `hexacopter_sim` | Gazebo world, `ros_ign_bridge` setup, `sim.launch.py`, `full_mission.launch.py` |
| `hexacopter_control` | The five mission nodes, `hexa_common.py`, RViz config, `mission.launch.py` |

## Node graph

```
wp --/sweep/done--> mapper --/mapping/final, /map_inflated--> planner --/plan/path--> follower
                    victim_detector --/victims, /victims/final-->  planner, follower
```

| Node | In | Out |
|---|---|---|
| `wp` | `/model/X4/odometry` | `/X4/gazebo/command/twist`, `/X4/enable`, `/sweep/done` |
| `mapper` | `/camera/image_raw`, `/camera/camera_info`, `/model/X4/odometry`, `/sweep/done` | `/map`, `/map_inflated`, `/mapping/final`, `/debug/obstacle_mask` |
| `victim_detector` | camera, odometry, `/sweep/done` | `/victims`, `/victims_markers`, `/victims/final`, `/debug/victims` |
| `planner` | `/map_inflated`, `/victims`, `/mapping/final`, `/victims/final`, `/sweep/done`, `/replan` | `/plan/path`, `/plan/markers` |
| `follower` | `/plan/path`, `/victims`, odometry | `/X4/gazebo/command/twist`, `/drone/pose`, `/drone/trail`, `/mission/complete` |

Latched topics (`/sweep/done`, `/mapping/final`, `/victims/final`, `/plan/path`) mean the nodes can start in any order.
All nodes use the `map` frame, which equals the Gazebo world frame (the drone spawns at the origin).

## How it works

### Coverage sweep

The camera footprint on the ground at altitude $z$ with camera offset $d_z=0.08\,\mathrm{m}$ and field of view $\theta=1.0472\,\mathrm{rad}$ has swath width

$$w=2(z-d_z)\tan\left(\frac{\theta}{2}\right)$$

Lines are spaced $w(1-\text{overlap})$ apart with 20% overlap, and the ends of each line are inset so the footprint, not the drone, covers the arena edge.

### Mapping

Each sampled pixel gives a camera ray $\mathbf{d}$. With the camera at height $h$ above the plane $z=z_p$, the ray hits the plane at

$$\mathbf{x}=\mathbf{c}+t\,\mathbf{d},\qquad t=\frac{h}{-d_z}$$

Each ground cell collects votes from every view that covered it. Tall obstacles smear onto the ground behind them (parallax), so an obstacle vote is not as trustworthy as a ground vote. A cell is occupied only if

$$\frac{\text{obstacle votes}}{\text{total votes}}\ge0.8$$

and it needs at least 3 hits to count as known. This raised IoU on the synthetic test world from 0.79 to 0.90.
Frames are skipped while the drone is below 1.5 m or tilted more than 12 degrees, and each image is paired with the odometry pose at its own timestamp (`PoseBuffer`). Specks of 2 cells or fewer are removed, and obstacles are inflated by 0.35 m for planning.

### Victim detection

Red, green and blue blobs are found with HSV thresholds (minimum area 250 px, blobs touching the image border are dropped). Each centroid is projected onto the marker plate plane $z=0.01\,\mathrm{m}$. Sightings of the same colour within 0.35 m are merged, and a victim is confirmed after 3 observations.

### Planning

The planner visits victims nearest-first, then returns home. Each leg is an 8-connected A* search with no diagonal corner cutting and the octile heuristic

$$h=\Delta x+\Delta y+(\sqrt{2}-2)\min(\Delta x,\Delta y)$$

The result is smoothed by greedy line-of-sight shortcutting and resampled every 0.15 m. Unknown cells are treated as blocked, and a 1 m radius around the start is forced free because the take-off pad is never mapped well.

### Path following

A pure-pursuit carrot sits $L=0.6\,\mathrm{m}$ ahead along the path, and the commanded speed is

$$\mathbf{v}=v_c\frac{\mathbf{c}-\mathbf{p}}{\lVert\mathbf{c}-\mathbf{p}\rVert}$$

Within $L$ of the end it switches to proportional control, $\mathbf{v}=k_p(\mathbf{g}-\mathbf{p})$, capped at $v_c$. Altitude is held with a P controller, and velocities are rotated from the world frame to the body frame before they are sent. The drone hovers 1.5 s when it comes within 0.4 m of a victim, and cross-track error (max and RMS) is logged at the end.

## Parameters

| Node | Parameter | Default |
|---|---|---|
| `mapper` | `resolution` | 0.05 |
| `mapper` | `x_min`, `y_min`, `x_max`, `y_max` | -1.0, -7.0, 17.0, 7.0 |
| `mapper` | `inflate_radius` | 0.35 |
| `mapper` | `occupied_threshold` | 0.8 |
| `mapper` | `min_hits` | 3 |
| `mapper` | `save_dir` | `~` |
| `victim_detector` | `min_area` | 250.0 |
| `victim_detector` | `min_observations` | 3 |
| `victim_detector` | `merge_radius` | 0.35 |
| `planner` | `waypoint_spacing` | 0.15 |
| `planner` | `start_clear_radius` | 1.0 |
| `planner` | `return_home` | true |
| `follower` | `altitude` | 2.0 |
| `follower` | `speed` | 1.0 |
| `follower` | `lookahead` | 0.6 |
| `follower` | `victim_radius` | 0.4 |
| `follower` | `dwell` | 1.5 |

Give float parameters as floats in YAML (`2.0`, not `2`), or ROS 2 raises a type-mismatch error.

## Code layout

`hexa_common.py` holds all the maths (camera model, projection, grid voting, A*, path tracker) with no ROS dependency, only NumPy and OpenCV, so it can be unit-tested or reused on another robot. The nodes are thin ROS wrappers around it.

## Roadmap: map-based navigation for the hexapod

The saved map and victim list are meant to be reused by the legged robot:

- [ ] Hexapod model spawned in the same world, publishing odometry and accepting body-velocity commands
- [ ] Map loader that reads `hexacopter_map.yaml`, inflates it by the hexapod footprint and feeds the existing planner
- [ ] Ground follower that reuses `PathTracker` and sends velocity commands to the gait controller
- [ ] Localisation against the saved map for hardware (simulation can use ground-truth odometry)
