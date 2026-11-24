# Vision-Guided Box Hunting Drone (PX4 SITL + Gazebo)

A PX4 drone in Gazebo that uses its onboard camera to find a yellow target box
and fly into it. When the drone hits the box, the box disappears and spawns
again somewhere else. The drone then turns in place to scan for it, finds it
again, and flies at it, over and over.

It works in any Gazebo world PX4 ships with (`aruco`, `baylands`, `default`,
etc.), as long as the world name in the scripts matches the one you launch
(see [Changing the world](#changing-the-world)).

## How it works

There are three processes. They talk to each other through Gazebo transport,
a local UDP socket, and MAVLink:

```
          Gazebo (gz sim)
   ┌────────────────────────────────────────────┐
   │  x500_mono_cam_0 drone      target_box     │
   └──────┬──────────────────────────▲──────────┘
          │ camera image             │ create / remove
          │ (gz-transport)           │ + drone pose
          ▼                          │
   ┌─────────────┐            ┌──────┴───────┐
   │  camera.py  │            │ spawn_box.py │
   │  (OpenCV)   │            │ (box manager)│
   └──────┬──────┘            └──────────────┘
          │ JSON over UDP 127.0.0.1:6969
          ▼
   ┌──────────────────┐   MAVLink (udp 14540)   ┌───────────┐
   │ mav_sdk/main.py  │ ──────────────────────► │ PX4 SITL  │
   │ (flight control) │   offboard velocities   └───────────┘
   └──────────────────┘
```

### 1. Box manager: `spawn_box.py`
- Spawns `target_box` (a 1 m yellow cube from `sdf_folder/target_box.sdf`) at
  the position in `box_config.json`.
- Subscribes to `/world/<world>/dynamic_pose/info` and tracks the drone's
  position.
- Once the drone is within **1.4 m** of the box center, it removes the box and
  spawns it again at a random `x, y` in `[-10, 10]` m. The height stays the same.

### 2. Vision: `camera.py`
- Subscribes to the drone's camera topic directly through gz-transport. No
  ROS and no bridge are needed.
- Finds the box with an **HSV color threshold** for yellow, cleans up the mask
  with a morphological open, and keeps the largest contour.
- Shows the live feed with a bounding box drawn over the target, and sends a
  JSON message to `127.0.0.1:6969` for every frame:
  ```json
  {"visible": true, "center_x": 640, "center_y": 360,
   "box_width": 42, "box_height": 40, "image_width": 1280, "image_height": 720}
  ```
  or `{"visible": false}` when the box isn't in view.

### 3. Flight control: `mav_sdk/main.py`
- Connects to PX4 through MAVSDK, arms, takes off, and climbs to **home + 20 m**.
- Switches to **offboard mode** and sends body-frame velocity and yaw-rate
  commands at about 10 Hz from a `VisualTracker` controller:
  - **Scanning:** if the box isn't visible, the drone yaws at 15 °/s to search.
  - **Centering:** the pixel error between the box center and the image
    center is normalized and smoothed (EMA, `alpha = 0.35`), then turned into
    a yaw rate (horizontal error) and a climb/descend speed (vertical error).
  - **Pursuing:** once the box stays inside the lock tolerance (about 8–10% of
    the frame), the drone flies forward at 2 m/s while it keeps correcting.
- It can also estimate distance from the box's pixel width using a pinhole
  camera model (`estimate_distance`), with the camera's 1.74 rad horizontal FOV
  and the known 1 m cube size.

When the drone hits the box, `spawn_box.py` moves it. The camera then stops
seeing it, so the controller goes back to scanning and the loop starts again.

## Files

| Path | Purpose |
|------|---------|
| `spawn_box.py` | Spawns the target box, watches the drone, and respawns the box on contact |
| `camera.py` | Gazebo camera viewer, yellow box detector, and UDP publisher |
| `mav_sdk/main.py` | MAVSDK offboard controller (takeoff, scan, center, pursue) |
| `box_config.json` | Starting box pose, SDF path, entity name, and drone model name |
| `sdf_folder/target_box.sdf` | 1×1×1 m static yellow box model |
| `requirements.txt` | pip packages (`mavsdk<4`, `opencv-python`, `numpy`) |
| `setup_container.sh` | Installs the Python dependencies in the `px4-dev` container |

## Setup

This setup runs inside the `px4-dev` Docker container, with PX4 mounted at
`/src/PX4-Autopilot`. On a brand-new container, first run PX4's
`Tools/setup/ubuntu.sh` to get the toolchain and Gazebo Harmonic. Then:

```bash
bash /src/PX4-Autopilot/scripts/setup_container.sh
```

This adds the Gazebo apt repo if needed, installs `python3-gz-transport13`,
installs the pip requirements, and checks that all the imports work.

> `mavsdk` is pinned to `<4`. Version 4 and later use a different native API
> in which `System()` no longer works.

## Running

Each step runs in its own shell inside the container.

**1. Start the simulation** with a camera-equipped drone. The plain `gz_x500`
has no camera.

```bash
cd /src/PX4-Autopilot
PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam
```

**2. Spawn the box.** Run this from the `scripts/` folder, because
`box_config.json` is loaded with a relative path:

```bash
cd /src/PX4-Autopilot/scripts
python3 spawn_box.py
```

**3. Start the vision node:**

```bash
python3 /src/PX4-Autopilot/scripts/camera.py
```

A "Drone Camera" window opens. Press `q` to close it.

**4. Start the flight controller:**

```bash
python3 /src/PX4-Autopilot/scripts/mav_sdk/main.py
```

The drone takes off, climbs, and starts hunting the box.

## Changing the world

The world name is hardcoded as `aruco` in a few places. To use another world
(for example `baylands`), change it in all of them so they match the
`PX4_GZ_WORLD` you launch:

- `spawn_box.py`: `/world/aruco/create`, `/world/aruco/remove`,
  `/world/aruco/dynamic_pose/info`
- `camera.py`: `CAMERA_TOPIC`

To check the camera topic while the sim is running:

```bash
gz topic -l | grep camera
```

## Tuning

| What | Where | Default |
|------|-------|---------|
| Yellow HSV range | `camera.py`: `YELLOW_LOWER` / `YELLOW_UPPER` | `[20,100,100]`–`[35,255,255]` |
| Minimum blob size | `camera.py`: `MIN_AREA_PX` | 30 px |
| Contact distance | `spawn_box.py`: `callback_function` | 1.4 m |
| Respawn range | `spawn_box.py`: `random_x_value` / `random_y_value` | ±10 m |
| Starting box pose | `box_config.json` | `(-5, 0, 20)` |
| Cruise altitude | `mav_sdk/main.py`: `target_alt` | home + 20 m |
| Yaw / altitude gains | `VisualTracker.k_p_yaw` / `k_p_alt` | 25.0 / 0.8 |
| Forward speed | `VisualTracker.compute_control` | 2.0 m/s |
| Lock tolerances | `VisualTracker.lock_enter_tol` / `lock_exit_tol` | 0.08 / 0.10 |
| Scan yaw rate | `VisualTracker.compute_control` | 15 °/s |

## Troubleshooting

- **No camera frames:** make sure you launched `gz_x500_mono_cam`, and that
  `CAMERA_TOPIC` matches the output of `gz topic -l`. Subscribing to a topic
  that doesn't exist still succeeds, so the script just warns after 5 s.
- **Box never spawns:** the world name in `spawn_box.py` must match the
  running world, and `sdf_filename` in `box_config.json` must be a valid path
  inside the container.
- **Box doesn't respawn on contact:** `DRONE_NAME` in `box_config.json` must
  match the drone's model name in Gazebo (default `x500_mono_cam_0`).
- **Box not detected:** adjust the HSV range. World lighting changes how the
  yellow looks.
