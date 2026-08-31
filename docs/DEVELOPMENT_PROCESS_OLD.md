## Development process

### Stage 1 — Official FR3 description

- Use `franka_description` rather than copying or modifying the official model.
- Verify `fr3_link0`, `fr3_hand_tcp`, joint states, and the TF tree.
- Visualize the robot in RViz before adding the environment.

cd + specific file location:

```bash
 cd ~/fr3_vision_sorting
```

```build the docker
docker compose up
```
```bash
docker exec -it fr3_vision_sorting bash
```
allow docker demonstrate the window
```bash
echo $DISPLAY
xhost +local:docker
```
Get int docker:
```bash
docker exec -it fr3_vision_sorting bash
```

```bash
source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash
```

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true
```

```bash
ros2 pkg prefix franka_description
ros2 run tf2_ros tf2_echo fr3_link0 fr3_hand_tcp
```
### Before Moving the Real FR3

1. Check Franka Desk
```bash
Make sure:

Control Box Connected
        ↓
Robot Unlocked
        ↓
FCI Enabled
        ↓
Robot Ready
```
2. Check ROS 2 Controller
```bash
ros2 control list_controllers
```
The arm controller should be:
```bash
active
```
Only start motion when Franka Desk + FCI + ROS 2 controller are ready.



### Stage 2 — Fixed-coordinate motion

Before vision, command a known target pose:

```text
Object position in fr3_link0: [0.45, 0.10, 0.03] m
```

#### Quick Command Reference
Terminal 1 — MoveIt
```bash
docker exec -it fr3_vision_sorting bash

source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash

ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true
```
#### Terminal 2 — Record Poses
docker exec -it fr3_vision_sorting bash
```
source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash

ros2 topic list | grep joint_states
ros2 topic hz /joint_states

mkdir -p /workspace/config

ros2 topic echo /joint_states --once \
  > /workspace/config/home_joint_state.yaml

ros2 topic echo /joint_states --once \
  > /workspace/config/pre_grasp_joint_state.yaml
```

Check:
```bash
ls -lh /workspace/config
Build Your Package
cd /workspace/ros2_ws
```
```bash
source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash
```
```bash
colcon build --symlink-install
source install/setup.bash
```bash

Key Principle:

Always develop in this order:
```bash
Simple
  ↓
Repeatable
  ↓
Safe
  ↓
Fake Hardware
  ↓
Real Hardware
  ↓
Vision
  ↓
Autonomous Operation
```
Never test a new motion for the first time at full speed on the real FR3.



Complete:

```text
HOME → PRE_GRASP → OPEN → DESCEND → CLOSE
     → LIFT → MOVE_TO_BIN → RELEASE → HOME
```

Do not add vision until this sequence succeeds ten times.

### Stage 3 — Planning Scene

Add collision objects for:

- table;
- camera stand;
- sorting bins;
- detected object.

When the object is grasped, attach it to `fr3_hand_tcp`. Detach it after release.

### Stage 4 — Gazebo sorting cell

Create a simulation containing:

- official FR3 model;
- table;
- fixed overhead RGB-D camera;
- red cube;
- green cylinder;
- blue box;
- three destination containers.

### Stage 5 — OpenCV detection

Initial algorithm:

```text
RGB → HSV → color mask → morphology
    → contours → center pixel (u,v)
```

Use OpenCV before YOLO because it is easier to debug lighting, image topics, coordinates, and depth alignment.

### Stage 6 — Depth-based 3D position

For pixel `(u,v)`, depth `Z`, and camera intrinsics:

```math
X = \frac{(u-c_x)Z}{f_x}
```

```math
Y = \frac{(v-c_y)Z}{f_y}
```

Use the median depth from a small region rather than one pixel.

### Stage 7 — TF2 transformation

Transform:

```text
camera_color_optical_frame → fr3_link0
```

Display the result as an RViz marker. The marker must appear on the object before the pose is sent to MoveIt.

### Stage 8 — Eye-to-hand calibration

For the fixed overhead RealSense, estimate:

```math
{}^{base}T_{camera}
```

Use an AprilTag or ChArUco target and collect 15–25 varied robot poses. Validate on independently measured positions.

Target accuracy:

```text
Mean 3D error:    < 10 mm
Maximum 3D error: < 20 mm
```

### Stage 9 — Real FR3 transfer

Before the first real movement:

- complete at least 30 successful simulation cycles;
- set velocity and acceleration scaling to 0.10 or lower;
- enforce a Cartesian workspace boundary;
- add all fixed obstacles to the Planning Scene;
- keep the enable device and emergency stop accessible;
- begin with `HOME → PRE_GRASP → HOME`;
- do not begin with continuous sorting.

## Planned ROS package summary

| Module | Responsibility |
|---|---|
| `color_detector.py` | Detect object class and center pixel |
| `depth_to_point.py` | Convert aligned depth into camera-frame 3D position |
| `object_pose_transformer.py` | Transform the pose into `fr3_link0` |
| `planning_scene_node.py` | Maintain table, bins, objects and attachments |
| `pick_place_node.py` | Plan and execute the sorting sequence |
| `metrics_logger.py` | Record error, success rate and cycle time |
| Launch files | Start simulation, vision or the real-robot configuration |

## Key technologies

| Technology | Purpose |
|---|---|
| ROS 2 Jazzy | Nodes, topics, parameters, actions and system integration |
| Franka ROS 2 | FR3 description, hardware interface and controllers |
| Gazebo Harmonic | Robot, object, contact and RGB-D simulation |
| MoveIt 2 / MoveItPy | Collision-aware manipulation planning |
| Intel RealSense | Physical RGB-D perception |
| OpenCV | Initial color and contour detection |
| TF2 | Camera-to-robot coordinate transformation |
| Docker Compose | Reproducible environment, USB access and GUI configuration |
| Python 3.12 | Perception, planning orchestration and metrics |

## Experiment metrics

Save one row for every trial:

```csv
trial_id,class,predicted_x,predicted_y,predicted_z,ground_truth_x,ground_truth_y,ground_truth_z,position_error_mm,grasp_success,cycle_time_s,failure_reason
1,red_cube,0.451,0.098,0.031,0.450,0.100,0.030,2.45,true,11.8,
```

Final targets:

| Metric | Target |
|---|---:|
| Mean localization error | Below 10 mm |
| Grasp success rate | Above 90% |
| Mean cycle time | Below 15 s |
| Unsafe target executions | 0 |
