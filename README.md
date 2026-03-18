# Drawing 2D Shapes in 3D Space with xArm7

A ROS 2 Humble package that commands a simulated UFactory xArm7 (7-DoF) robot to trace 2D shapes in 3D space using MoveIt 2.

## Table of Contents

- [Quick Start](#quick-start)
- [Detailed Setup](#detailed-setup)
- [Running the Shapes](#running-the-shapes)
- [Shape Definitions](#shape-definitions)
- [Approach](#approach)
- [Package Structure](#package-structure)
- [Features Implemented](#features-implemented)
- [Validation](#validation)

---

## Quick Start

If you already have the Docker container running and the package deployed:

```bash
# Inside the Docker container:
source /opt/ros/humble/setup.bash
source ~/xarm_ws/install/setup.bash
source ~/dev_ws/install/setup.bash

ros2 launch avatar_challenge start.launch.py
```

---

## Detailed Setup

### Prerequisites

- Docker installed ([Install Docker Engine](https://docs.docker.com/engine/install/))
- An RDP client (e.g. Remmina on Linux, mstsc on Windows)
- The challenge Docker image: `avatarrobotics/ros-humble-xarm:20250602`

### Step 1: Pull and Start the Docker Container

```bash
# Pull the Docker image (first time only)
docker pull avatarrobotics/ros-humble-xarm:20250602

# Create and start the container
# Maps port 5566 on host → port 3389 (RDP) inside the container
docker run -d \
  --name xarm-container \
  --platform linux/amd64 \
  -p 5566:3389 \
  avatarrobotics/ros-humble-xarm:20250602

# If the container already exists, just start it:
docker start xarm-container
```

Verify the container is running:

```bash
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
# Expected: xarm-container   Up ...   0.0.0.0:5566->3389/tcp
```

### Step 2: Deploy the Code into the Container

```bash
# Clone this repository (if you haven't already)
git clone <this-repo-url>
cd <repo-directory>

# Remove any old version inside the container
docker exec xarm-container \
  rm -rf /home/dev/dev_ws/src/avatar_challenge \
         /home/dev/dev_ws/build/avatar_challenge \
         /home/dev/dev_ws/install/avatar_challenge

# Copy the package into the container's ROS 2 workspace
docker cp src/avatar_challenge \
  xarm-container:/home/dev/dev_ws/src/avatar_challenge

# Fix file permissions
docker exec xarm-container \
  chown -R dev:dev /home/dev/dev_ws/src/avatar_challenge
docker exec xarm-container \
  chmod +x /home/dev/dev_ws/src/avatar_challenge/scripts/shape_drawer.py
```

### Step 3: Build the ROS 2 Package Inside the Container

```bash
docker exec -u dev -w /home/dev/dev_ws xarm-container \
  bash -c "source /opt/ros/humble/setup.bash && \
           colcon build --packages-select avatar_challenge"
```

Expected output:

```
Starting >>> avatar_challenge
Finished <<< avatar_challenge [~1s]
Summary: 1 package finished
```

### Step 4: Connect to the Desktop via RDP

The container provides an XFCE desktop via xRDP on port 5566.

**Linux (e.g. using Remmina):**

```bash
# Install Remmina if not present
sudo apt install remmina remmina-plugin-rdp

# Connect
remmina -c rdp://localhost:5566
```

**Windows:**

```powershell
mstsc /v:localhost:5566 /w:1920 /h:1080
```

**Login credentials:**

| Field    | Value              |
|----------|--------------------|
| Address  | `localhost:5566`   |
| Username | `dev`              |
| Password | `ecstatic-robots`  |

After login, you will see an XFCE desktop environment with a taskbar at the bottom.

---

## Running the Shapes

### Option A: One-Click Launch (All Shapes)

Open a terminal inside the RDP desktop, then run:

```bash
source /opt/ros/humble/setup.bash
source ~/xarm_ws/install/setup.bash
source ~/dev_ws/install/setup.bash

ros2 launch avatar_challenge start.launch.py
```

This starts the MoveIt simulation, RViz, and the shape drawer node together.
The drawer will execute all shapes defined in `config/shapes.yaml` sequentially.

### Option B: Test Shapes Individually (Recommended)

This approach is better for debugging — keep the simulation running and test one shape at a time.

**Terminal 1 — Start the MoveIt simulation (keep open):**

```bash
source /opt/ros/humble/setup.bash
source ~/xarm_ws/install/setup.bash
ros2 launch xarm_moveit_config xarm7_moveit_fake.launch.py
```

Wait until RViz opens and the robot model is displayed.

**Terminal 2 — Run shapes one by one:**

```bash
source /opt/ros/humble/setup.bash
source ~/xarm_ws/install/setup.bash
source ~/dev_ws/install/setup.bash

# Shape 1: Big square (25cm sides)
ros2 run avatar_challenge shape_drawer.py \
  --ros-args -p config_file:=/home/dev/dev_ws/install/avatar_challenge/share/avatar_challenge/config/shape_1_square.yaml

# Shape 2: Equilateral triangle (12cm sides)
ros2 run avatar_challenge shape_drawer.py \
  --ros-args -p config_file:=/home/dev/dev_ws/install/avatar_challenge/share/avatar_challenge/config/shape_2_triangle.yaml

# Shape 3: Full circle (6cm radius, arc interpolation)
ros2 run avatar_challenge shape_drawer.py \
  --ros-args -p config_file:=/home/dev/dev_ws/install/avatar_challenge/share/avatar_challenge/config/shape_3_circle.yaml

# Shape 4: Regular pentagon on tilted plane (with blending)
ros2 run avatar_challenge shape_drawer.py \
  --ros-args -p config_file:=/home/dev/dev_ws/install/avatar_challenge/share/avatar_challenge/config/shape_4_pentagon.yaml

# Shape 5: Large diamond (20cm diagonals)
ros2 run avatar_challenge shape_drawer.py \
  --ros-args -p config_file:=/home/dev/dev_ws/install/avatar_challenge/share/avatar_challenge/config/shape_5_diamond.yaml

# Shape 6: Large hexagon (15cm radius, with blending)
ros2 run avatar_challenge shape_drawer.py \
  --ros-args -p config_file:=/home/dev/dev_ws/install/avatar_challenge/share/avatar_challenge/config/shape_6_hexagon.yaml

# Shape 7: 5-pointed star (20cm span, pitch-tilted)
ros2 run avatar_challenge shape_drawer.py \
  --ros-args -p config_file:=/home/dev/dev_ws/install/avatar_challenge/share/avatar_challenge/config/shape_7_star.yaml
```

Wait for each shape to output `✓ All shapes executed!` before running the next one.
The RViz window stays open throughout — no need to restart the simulation between shapes.

### Using a Custom Shape Config

```bash
ros2 launch avatar_challenge start.launch.py config_file:=/path/to/my_shapes.yaml

# Or with ros2 run:
ros2 run avatar_challenge shape_drawer.py \
  --ros-args -p config_file:=/path/to/my_shapes.yaml
```

Or edit `config/shapes.yaml` directly, then rebuild:

```bash
cd ~/dev_ws && colcon build --packages-select avatar_challenge
```

---

## Shape Definitions

Shapes are defined in YAML format. See [`config/shapes.yaml`](src/avatar_challenge/config/shapes.yaml) for a complete example. Individual shape files are also provided in the `config/` directory for per-shape testing.

### Parameters

| Parameter     | Type   | Description                                                          |
|---------------|--------|----------------------------------------------------------------------|
| `name`        | string | Human-readable label                                                 |
| `type`        | string | `"polygon"` (default) or `"arc"`                                     |
| `vertices`    | list   | 2D `[u, v]` vertex pairs (polygon only). **First vertex must be `[0, 0]`** |
| `position`    | list   | `[x, y, z]` origin in robot base frame (metres)                     |
| `orientation` | list   | `[roll, pitch, yaw]` defining the 3D drawing plane (radians)        |
| `closed`      | bool   | Whether to connect last vertex back to first (default: `true`)      |
| `blend`       | bool   | Use finer interpolation for smoother curves (default: `false`)      |

**Arc-specific parameters** (when `type: "arc"`):

| Parameter         | Type  | Description                                  |
|-------------------|-------|----------------------------------------------|
| `arc_center`      | list  | `[u, v]` centre on the 2D plane              |
| `arc_radius`      | float | Arc radius in metres                         |
| `arc_start_angle` | float | Start angle in radians                       |
| `arc_end_angle`   | float | End angle in radians (2π for full circle)    |
| `arc_num_points`  | int   | Number of interpolation points (default: 48) |

### Example: Polygon (Square)

```yaml
shapes:
  - name: "square_45deg"
    type: "polygon"
    vertices:
      - [0.0, 0.0]
      - [0.0, 0.1]
      - [0.1, 0.1]
      - [0.1, 0.0]
    position: [0.30, 0.0, 0.30]
    orientation: [0, 0, 0.7854]  # 45° yaw rotation
    closed: true
```

### Example: Arc (Full Circle)

```yaml
shapes:
  - name: "circle"
    type: "arc"
    arc_center: [0.0, 0.0]
    arc_radius: 0.06
    arc_start_angle: 0
    arc_end_angle: 6.2832        # 2π = full circle
    arc_num_points: 48
    position: [0.30, 0.18, 0.30]
    orientation: [0, 0, 0]
    closed: true
```

### Pre-built Shape Configs

| File                    | Shape             | Key Properties                        |
|-------------------------|-------------------|---------------------------------------|
| `shape_1_square.yaml`  | 25cm square       | Basic polygon                         |
| `shape_2_triangle.yaml`| 12cm triangle     | Equilateral, closed                   |
| `shape_3_circle.yaml`  | 6cm radius circle | Arc type, 48 interpolation points     |
| `shape_4_pentagon.yaml`| Pentagon          | Tilted plane (17° roll) + blending    |
| `shape_5_diamond.yaml` | 20cm diamond      | Large polygon                         |
| `shape_6_hexagon.yaml` | 15cm hexagon      | 30° yaw rotation + blending           |
| `shape_7_star.yaml`    | 5-pointed star    | Complex vertices, pitch-tilted plane  |
| `shapes.yaml`          | All 7 combined    | Full test suite                       |

---

## Approach

### Problem Analysis

The challenge requires tracing 2D shapes on arbitrarily oriented planes in 3D space. This breaks down into three sub-problems:

1. **Coordinate Transformation** — Map 2D vertices `(u, v)` to 3D positions via rotation matrix and translation
2. **Path Planning** — Compute smooth, collision-free trajectories through the 3D waypoints
3. **Motion Execution** — Send trajectories to the robot controller for execution

### Two-Phase Motion Strategy

For each shape, the robot performs two distinct phases of motion:

```
Phase 1: Move to Start Point
  ├── Uses MoveGroup action (joint-space planning, OMPL RRTConnect)
  ├── Robot moves from current pose to the shape's first vertex
  └── Path may look indirect — RRTConnect only guarantees collision-free arrival

Phase 2: Trace the Shape
  ├── Uses GetCartesianPath service (Cartesian straight-line planning)
  ├── End-effector follows precise straight-line segments along shape edges
  └── If coverage < 80%, falls back to per-waypoint joint-space moves
```

### Key Design Decisions

| Decision                | Choice                                           | Rationale                                                                         |
|-------------------------|--------------------------------------------------|-----------------------------------------------------------------------------------|
| MoveIt interface        | Direct ROS 2 service/action clients              | `moveit_commander` is not available in the container; direct clients are reliable  |
| Shape tracing           | Cartesian path (`GetCartesianPath`)              | Drawing requires straight-line end-effector motion between vertices                |
| Start-point motion      | MoveGroup action (joint-space)                   | Joint-space planning is faster and avoids singularities for large moves            |
| Fallback strategy       | Per-waypoint MoveGroup if Cartesian coverage < 80% | Ensures shapes are still drawn even in challenging configurations               |
| EE orientation          | Perpendicular to drawing plane (π flip about X)  | The tool must "face" the drawing surface                                          |
| Speed                   | 20% velocity/acceleration scaling                | Slower motion for clear visual observation                                        |

### Algorithm

For each shape defined in the YAML config:

```
1. Parse shape definition (vertices or arc parameters)
2. Build rotation matrix R = Rz(yaw) × Ry(pitch) × Rx(roll)
3. For each 2D vertex (u, v):
     p_3d = origin + R × [u, v, 0]
4. Compute end-effector orientation: q = R_shape × R_flip(π, 0, 0)
5. If closed, append first vertex to close the shape
6. Move to first waypoint (joint-space, MoveGroup action)
7. Compute Cartesian path through remaining waypoints
8. If fraction ≥ 80%: execute trajectory
   Else: fall back to per-waypoint MoveGroup moves
```

### Assumptions

- The robot starts in a reachable default pose
- All target positions are within the xArm7 workspace (≈ 0.7m reach)
- The `xarm7_moveit_fake.launch.py` provides all required MoveIt 2 services
- Shape vertices define reasonable geometry that doesn't cause self-collision

---

## Package Structure

```
src/avatar_challenge/
├── CMakeLists.txt            # Build configuration
├── package.xml               # ROS 2 package manifest (ament_cmake)
├── config/
│   ├── shapes.yaml           # Combined shape definitions (all 7)
│   ├── shape_1_square.yaml   # Individual: 25cm square
│   ├── shape_2_triangle.yaml # Individual: 12cm equilateral triangle
│   ├── shape_3_circle.yaml   # Individual: 6cm circle (arc type)
│   ├── shape_4_pentagon.yaml # Individual: tilted pentagon + blend
│   ├── shape_5_diamond.yaml  # Individual: 20cm diamond
│   ├── shape_6_hexagon.yaml  # Individual: 15cm hexagon + blend
│   └── shape_7_star.yaml    # Individual: 5-pointed star
├── launch/
│   └── start.launch.py       # Launch file (MoveIt sim + shape drawer)
└── scripts/
    └── shape_drawer.py       # Core ROS 2 node (~680 lines)
```

### Key Files

- **`shape_drawer.py`** — The main ROS 2 node. Loads YAML, validates shapes, performs 2D→3D coordinate transforms, publishes RViz markers, and executes motions via MoveIt 2.
- **`start.launch.py`** — Launches the xArm7 MoveIt fake simulation and delays the shape drawer by 10 seconds to allow MoveIt to fully initialise.
- **`shapes.yaml`** — Contains all 7 shape definitions. Individual YAML files are provided for per-shape testing.

---

## Features Implemented

| Feature                    | Status | Details                                                   |
|----------------------------|--------|-----------------------------------------------------------|
| Core: polygon drawing      | ✅     | Square, triangle, pentagon, diamond, hexagon, star        |
| Core: 3D plane projection  | ✅     | RPY rotation matrix + translation                         |
| Core: trajectory planning  | ✅     | Cartesian path with automatic fallback                    |
| Bonus: RViz visualization  | ✅     | LINE_STRIP markers for shape outlines + EE trail          |
| Bonus: circular arcs       | ✅     | Parametric angle interpolation with configurable points   |
| Bonus: segment blending    | ✅     | Finer eef_step (0.002m) for smoother motion               |
| Bonus: B-splines           | ❌     | Not implemented                                           |

---

## Validation

### Expected Terminal Output

Each shape should produce output similar to:

```
[shape_drawer] Config: .../shape_1_square.yaml
[shape_drawer] Loaded 1 shape(s)
[shape_drawer]   → "big_square": 5 waypoints
[shape_drawer] All MoveIt services/actions are ready.
[shape_drawer] Published 1 shape markers
[shape_drawer] ▶ "big_square" — 5 waypoints
[shape_drawer]   Moving to start of "big_square" …
[shape_drawer]   Cartesian plan: 100.0%
[shape_drawer]   ✓ "big_square" executed
[shape_drawer] ✓ All shapes executed!
```

### What to Observe in RViz

| Item                 | Expected                                            |
|----------------------|-----------------------------------------------------|
| Robot model          | Orange xArm7 robot at scene centre                  |
| Shape outlines       | Coloured LINE_STRIP markers in front of the robot   |
| Robot arm motion     | End-effector traces along the shape edges precisely  |
| Cartesian coverage   | 100% for all shapes (logged in terminal)            |

### Verified Shapes

| Shape              | Waypoints | Cartesian Coverage | Status |
|--------------------|-----------|-------------------|--------|
| big_square         | 5         | 100%              | ✅     |
| triangle           | 4         | 100%              | ✅     |
| circle             | 49        | 100%              | ✅     |
| pentagon_tilted    | 6         | 100%              | ✅     |
| large_diamond      | 5         | 100%              | ✅     |
| large_hexagon      | 7         | 100%              | ✅     |
| large_star         | 11        | 100%              | ✅     |
