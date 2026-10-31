# Autonomous Differential-Drive Robot

A differential-drive robot with a 2D LiDAR and an RGB camera, driving on a road in Gazebo Classic.

- **Autonomous driving:** Nav2 — A* planning (`SmacPlanner2D`) and Pure Pursuit tracking (`RegulatedPurePursuitController`), localization with AMCL.
- **Manual driving:** PyQt5 control panel (arrow keys / WASD, RPM limit, smooth acceleration).
- **Mode switching:** one key in the panel toggles MANUAL ↔ AUTO. There is no priority; only the active mode's command reaches the robot.
- **Pose printing:** one key in the panel prints the robot's current pose to the terminal.
- The whole system starts with a single launch file.

![](/home/arda/rake_ws/gazebo.png)

![](/home/arda/rake_ws/rviz.png)

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11

System packages are installed with `rosdep` (below). To install them manually:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-tf2-ros-py python3-pyqt5
```

## Installation and Build

```bash
git clone https://github.com/ardaElibol675/Differential-Drive.git rake_ws
cd rake_ws
source /opt/ros/humble/setup.bash

# Dependencies (on first use run: sudo rosdep init && rosdep update)
rosdep install --from-paths src -y --ignore-src

colcon build --symlink-install
source install/setup.bash
```

## Running

```bash
ros2 launch robot_bringup bringup.launch.py
```

**Autonomous driving:**

1. Click the control panel window and press `x` to switch to **AUTO** (the system starts in MANUAL).
2. Set a goal on the road with the **Nav2 Goal** tool in RViz. The robot follows the plan (`/plan`) to the goal. The plan is cleared from RViz when the goal is reached.
3. Switching to MANUAL with `x` while driving stops the robot, cancels the Nav2 goal and clears the plan in RViz. Set a new goal after switching back to AUTO.
4. A goal set in MANUAL mode is planned and shown, but not driven. Switching to AUTO with `x` starts following it.

Goal progress is printed to the terminal (and shown in the panel's State box) with a `[NAV]` prefix:

| Message                                                   | When                                         |
| --------------------------------------------------------- | -------------------------------------------- |
| `Goal received`                                           | A Nav2 goal is accepted                      |
| `Route created, driving`                                  | Plan ready, mode is AUTO                     |
| `Route created, waiting for AUTO`                         | Plan ready, mode is MANUAL; goal is held     |
| `Driving started`                                         | Switched to AUTO with a held goal            |
| `Route canceled (MANUAL)`                                 | Switched to MANUAL while driving             |
| `Goal reached`                                            | Goal reached within tolerance                |
| `Route canceled` / `Navigation failed` / `No route found` | Other endings (e.g. RViz Cancel, Nav2 abort) |

### Launch arguments

| Argument                  | Default                           | Description                                                               |
| ------------------------- | --------------------------------- | ------------------------------------------------------------------------- |
| `slam`                    | `false`                           | `true`: slam_toolbox (mapping) instead of Nav2                            |
| `map`                     | `robot_navigation/maps/road.yaml` | Navigation map                                                            |
| `world`                   | `robot_gazebo/worlds/road.world`  | Gazebo world                                                              |
| `gui`                     | `true`                            | `false`: gzserver only (no Gazebo window)                                 |
| `use_rviz`                | `true`                            | Open RViz                                                                 |
| `use_teleop`              | `true`                            | Open the control panel                                                    |
| `show_logs`               | `false`                           | `true`: print all node output to the terminal                             |
| `x_pose`, `y_pose`, `yaw` | `0.0`                             | Spawn pose (also used as the AMCL initial pose). Use decimals, e.g. `1.0` |

## Keybinds

All keys work **while the control panel window has focus**. When focus is lost, held drive keys are treated as released.

| Key       | Function                        |
| --------- | ------------------------------- |
| `↑` / `W` | Move Forward                    |
| `↓` / `S` | Move Backward                   |
| `←` / `A` | Turn Anti-Clockwise             |
| `→` / `D` | Turn Clockwise                  |
| `x`       | Toggle MANUAL ↔ AUTO            |
| `z`       | Prints the pose to the terminal |

Drive keys can be combined (e.g. `↑` + `→` for a right curve). The maximum wheel speed is set with the RPM slider in the panel. Drive keys only move the robot in MANUAL mode.

The mode can also be switched from the terminal:

```bash
ros2 service call /mode_manager/toggle_mode std_srvs/srv/Trigger
```

## Rebuilding the Map

```bash
ros2 launch robot_bringup bringup.launch.py slam:=true
```

Drive along the whole road with the panel (MANUAL mode), then save the map from another terminal:

```bash
ros2 run nav2_map_server map_saver_cli -f src/robot_navigation/maps/road
```

Then rebuild with `colcon build --symlink-install` and start in navigation mode.

## Packages

| Package             | Contents                                                                  |
| ------------------- | ------------------------------------------------------------------------- |
| `robot_description` | URDF/xacro model, Gazebo plugins, `display.launch.py`                     |
| `robot_gazebo`      | `road.world`, `sim.launch.py` (Gazebo + robot_state_publisher + spawn)    |
| `robot_navigation`  | Nav2 and slam_toolbox parameters, map, navigation/slam launch files       |
| `robot_tools`       | `mode_manager` (MANUAL/AUTO), `control_panel` (PyQt5 driving, mode, pose) |
| `robot_bringup`     | `bringup.launch.py`, panel/mode manager settings, RViz config             |

### Velocity command flow

```
control_panel ──/cmd_vel_teleop──┐
                                 ├─► mode_manager ──/cmd_vel──► Gazebo diff_drive
controller_server ─┐             │   (active mode only)
behavior_server  ──┴─/cmd_vel_raw─► velocity_smoother ─/cmd_vel_nav─┘
```

The robot frame is `base_footprint` (center of the wheel axle). TF chain: `map → odom → base_footprint → base_link → sensors`.

## Troubleshooting

- **Gazebo does not open / "entity already exists":** A `gzserver` from a previous run is still alive. Find it with `ps -C gzserver,gzclient -o pid,cmd` and stop it with `kill -9 <pid>`.
- **Drive keys are pressed but the robot does not move:** The mode is AUTO. Press `x` in the panel to switch to MANUAL.
- **The robot does not move after switching back to AUTO:** The previous goal was canceled when switching to MANUAL. Set a new goal. (A goal *set* in MANUAL is kept and resumes on AUTO.)
- **The panel does not receive keys:** The panel window does not have focus. Click the window.
