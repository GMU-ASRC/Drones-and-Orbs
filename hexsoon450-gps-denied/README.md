# hexsoon450-gps-denied

ROS 2 packages and flight control for the Hexsoon 450 GPS-denied UAV platform.

## Packages

### `hexsoon_bringup`

Top-level launch package that starts the entire stack with a single command.

```bash
ros2 launch hexsoon_bringup bringup.launch.py
```

Launch arguments:

| Argument | Default | Description |
|---|---|---|
| `fcu_url` | `serial:///dev/ttyACM0:115200` | FCU connection URL for OrangeCube |
| `use_web_video_server` | `true` | Enable MJPEG browser stream on port 8080 |

### `my_vio_launch`

Core launch package that brings up all subsystem nodes:

- **RealSense D435i** — color camera + merged IMU stream
- **OpenVINS (ov_msckf)** — visual-inertial odometry
- **MAVROS** — flight controller bridge
- **Pose Converter** — converts OpenVINS `PoseWithCovarianceStamped` to MAVROS `PoseStamped`
- **RTAB-Map** — occupancy grid mapping from VIO odometry
- **Web Video Server** — MJPEG camera stream on port 8080

### `flight_tests`

Pre-built flight test missions triggered from the dashboard UI or via ROS topic. All parameters are configurable at runtime — no rebuild required.

| Test | Command syntax | Description |
|---|---|---|
| 1 — Takeoff & Land | `quick_land:<alt_m>` | Arms, takes off to `alt_m`, lands immediately |
| 2 — Hover | `hover:<sec>:<alt_m>` | Arms, takes off, hovers for `sec` seconds, lands |
| 3 — Move & Return | `move_return:<alt_m>:<dist_m>` | Arms, takes off, moves `dist_m` forward (+X ENU), returns to start, lands |
| 4 — Spiral Search | `spiral_search:<alt_m>:<width_ft>:<height_ft>` | Arms, takes off, flies lawnmower pattern over specified area, detects ArUco marker (DICT_6X6_250) via downward camera, centers, and precision-lands on target |
| 5 — Follow | `follow:<target_id>:<alt_m>` | Arms, takes off, continuously follows an ArUco marker at fixed altitude, lands on command |
| 6 — Timed Follow & Land | `follow_timed:<target_id>:<alt_m>:<duration_s>` | Arms, takes off, follows ArUco marker for `duration_s` seconds building velocity estimate, then descends using predictive lookahead to land on a moving UGV |
| Abort | `abort` | Cancels any running test and lands immediately |

#### ArUco detection

- Dictionary: `DICT_6X6_250`
- Camera: `/dev/video6` (downward-facing)
- Marker size: 0.15 m
- Detection pipeline: CLAHE preprocessing → `detectMarkers` → corner-based `m_per_px` scale → ENU world coordinates
- Camera mount offset: 58.6 mm westward from body centre, corrected in position estimates

#### Timed Follow & Land (Test 6) — moving UGV landing

Two-phase approach to land on a moving UGV:

1. **Follow phase** — tracks the tag via EMA setpoint filter and builds an EMA velocity estimate over `duration_s` seconds so the velocity has converged before descent begins.
2. **Descent phase** — simultaneously lowers altitude at `DESCENT_RATE` m/s and commands a predictive setpoint ahead of the tag:

```
lookahead_t = min(alt / DESCENT_RATE, MAX_LOOKAHEAD_S)
target = setpoint + tag_velocity × lookahead_t
```

The lookahead automatically scales: farther ahead when higher (more time for UGV to move), converging to zero as the drone nears the ground. Native `LAND` mode is engaged at `LAND_ALT = 0.20 m`.

#### Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/flight_test/command` | `std_msgs/String` | SUB | Send a test command string |
| `/flight_test/status` | `std_msgs/String` | PUB | Human-readable status updates |
| `/flight_test/plan` | `std_msgs/String` | PUB | JSON flight plan with waypoints, active index, UAV position, marker detection |
| `/flight_test/aruco_tag` | `std_msgs/String` | PUB | JSON ArUco detection: `{id, x, y}` in ENU metres |
| `/flight_test/event` | `std_msgs/String` | PUB | JSON flight events: `{type, ...}` |

#### Flight events

| `type` | Extra fields | When emitted |
|---|---|---|
| `search_start` | `pattern`, `alt`, `area_w`, `area_h`, `waypoints` | Test 4 begins sweep |
| `landed_on_tag` | `id`, `x`, `y` | Touchdown confirmed on ArUco tag (Tests 4, 5, 6) |

### `uav_dashboard`

Real-time web dashboard served over HTTP (port 80) with a WebSocket data bridge (port 9090). LAN-only — no internet required.

Open in any browser on the same network:

```
http://<companion-ip>
```

#### Pages

| Page | URL | Description |
|---|---|---|
| Dashboard | `/index.html` | Live telemetry, camera feed, odometry, IMU, event log, MAV console |
| Flight Tests | `/flight_tests.html` | Run and configure all six flight test missions |
| System | `/system.html` | Host system info — CPU, RAM, disk, network interfaces, ROS nodes |
| Utils | `/utils.html` | Unit converter, command reference, ROS 2 quick reference, key topics, API & WebSocket reference |

#### Dashboard panels

- System status (MAV state, armed, mode, battery, VIO tracking)
- Live camera feed (MJPEG via web_video_server on port 8080)
- MAVROS odometry (global position + relative altitude)
- VIO local odometry (position + orientation)
- IMU sensor data (angular velocity + linear acceleration)
- System event log (aggregated `/rosout`)
- MAV command console

#### MAV command console

| Command | Syntax | Description |
|---|---|---|
| Arm | `arm` | Arm the vehicle |
| Disarm | `disarm` | Disarm the vehicle |
| Set mode | `mode <MODE>` | Set flight mode — `GUIDED`, `LOITER`, `LAND`, `STABILIZE` |
| Takeoff | `takeoff <alt_m>` | Takeoff to altitude in metres |
| Land | `land` | Land at current position |
| Move | `move <x> <y> <z>` | Publish local ENU setpoint |
| Move + yaw | `move <x> <y> <z> <yaw>` | Publish setpoint with heading in degrees |
| Help | `help` | Print command list to console |

## HTTP REST API

Base URL: `http://<companion-ip>`

| Method | Path | Description |
|---|---|---|
| `GET` | `/` | Redirect to `/index.html` |
| `GET` | `/<file>` | Serve static dashboard files |
| `GET` | `/api/map` | Latest RTAB-Map occupancy grid as PNG |
| `GET` | `/api/aruco_tags` | All ArUco tags detected during the current mission |

### `GET /api/map`

Returns a PNG image of the current occupancy grid. Map cells: free = white (255), occupied = black (0), unknown = grey (128). Y-axis is flipped to match standard image coordinates (north = up).

Response headers:

```
Content-Type: image/png
X-Map-Width: <cells>
X-Map-Height: <cells>
X-Map-Resolution: <m/cell>
X-Map-Origin-X: <m>
X-Map-Origin-Y: <m>
```

Returns `{"error": "map not yet available"}` (JSON, 503) if no map has been received yet.

### `GET /api/aruco_tags`

Returns all ArUco markers detected during the current mission.

```json
{
  "tags": [
    {
      "id": 7,
      "x": 1.234,
      "y": -0.567,
      "ts": 1713456789.123,
      "map_x": 2.1,
      "map_y": -0.3,
      "px": 312,
      "py": 198
    }
  ]
}
```

| Field | Description |
|---|---|
| `id` | ArUco marker ID |
| `x`, `y` | ENU world coordinates (metres) |
| `ts` | Unix timestamp of last detection |
| `map_x`, `map_y` | Position relative to map origin (metres) — `null` if no map received |
| `px`, `py` | Pixel coordinates in the `/api/map` image — `null` if no map received |

## WebSocket API

Connect to `ws://<companion-ip>:9090` to receive live telemetry and flight events.

### Outbound frames (server → client)

| `type` | Key fields | Source topic |
|---|---|---|
| `state` | `armed`, `mode`, `connected` | `/mavros/state` |
| `battery` | `voltage`, `current`, `remaining` | `/mavros/battery` |
| `global_pos` | `lat`, `lon`, `alt`, `rel_alt`, `heading` | `/mavros/global_position/*` |
| `local_pos` | `x`, `y`, `z`, `vx`, `vy`, `vz` | `/mavros/local_position/pose` + velocity |
| `vio_odom` | `x`, `y`, `z`, `qx`, `qy`, `qz`, `qw`, `vx`, `vy`, `vz` | `/ov_msckf/odomimu` |
| `imu` | `ax`, `ay`, `az`, `gx`, `gy`, `gz` | `/camera/camera/imu` |
| `rosout` | `level`, `name`, `msg` | `/rosout` |
| `flight_status` | `msg` | `/flight_test/status` |
| `flight_plan` | `waypoints`, `active_index`, `uav_pos`, `marker` | `/flight_test/plan` |
| `aruco_tag` | `id`, `x`, `y`, `ts`, `map_x`, `map_y`, `px`, `py` | `/flight_test/aruco_tag` |
| `search_start` | `pattern`, `alt`, `area_w`, `area_h`, `waypoints` | `/flight_test/event` |
| `landed_on_tag` | `id`, `x`, `y` | `/flight_test/event` |
| `sys_info` | `cpu`, `ram`, `disk`, `uptime`, `ros_nodes` | Internal (1 Hz) |

### Inbound frames (client → server)

| `type` | Fields | Effect |
|---|---|---|
| `command` | `cmd` | Publish `cmd` to `/flight_test/command` |
| `mav_console` | `cmd` | Parse and execute MAV console command |

## ROS 2 Topics

| Topic | Type | Direction |
|---|---|---|
| `/mavros/state` | `mavros_msgs/State` | SUB |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | SUB |
| `/mavros/global_position/rel_alt` | `std_msgs/Float64` | SUB |
| `/mavros/global_position/compass_hdg` | `std_msgs/Float64` | SUB |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | SUB |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | SUB |
| `/mavros/battery` | `sensor_msgs/BatteryState` | SUB |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | PUB |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | PUB |
| `/mavros/odometry/in` | `nav_msgs/Odometry` | PUB (VIO input) |
| `/ov_msckf/odomimu` | `nav_msgs/Odometry` | SUB |
| `/camera/camera/imu` | `sensor_msgs/Imu` | SUB |
| `/map` | `nav_msgs/OccupancyGrid` | SUB (RTAB-Map) |
| `/flight_test/command` | `std_msgs/String` | SUB |
| `/flight_test/status` | `std_msgs/String` | PUB |
| `/flight_test/plan` | `std_msgs/String` | PUB |
| `/flight_test/aruco_tag` | `std_msgs/String` | PUB |
| `/flight_test/event` | `std_msgs/String` | PUB |
| `/rosout` | `rcl_interfaces/Log` | SUB |

## MAVROS Services

| Service | Type | Purpose |
|---|---|---|
| `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | Arm / disarm |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | Set flight mode |
| `/mavros/cmd/takeoff` | `mavros_msgs/CommandTOL` | Takeoff |
| `/mavros/cmd/land` | `mavros_msgs/CommandTOL` | Land |
| `/mavros/mission/push` | `mavros_msgs/WaypointPush` | Upload waypoint mission |
| `/mavros/mission/clear` | `mavros_msgs/WaypointClear` | Clear waypoint mission |

## Quick Start

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this-repo> hexsoon450-gps-denied
cd ~/ros2_ws/src/hexsoon450-gps-denied
./stack_install.sh
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch hexsoon_bringup bringup.launch.py
```

Open `http://<companion-ip>` in a browser on the same LAN.
