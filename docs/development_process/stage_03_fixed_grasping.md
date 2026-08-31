# Stage 3 — Fixed-Position Grasping

Only begin after Stage 2 passes reliably. Stage 3 adds gripper control and two
new fixed arm poses, but it does not use RealSense coordinates yet.

## Objective

```text
HOME → PRE_GRASP → OPEN → GRASP → CLOSE → LIFT → HOME
```

Add and validate one action at a time. The program must stop before `LIFT` if
the gripper does not confirm a grasp.

## Required package structure

```text
ros2_ws/src/fr3_vision_sorting/
├── config/
│   └── fixed_grasp/
│       ├── home_joint_state.yaml
│       ├── pre_grasp_joint_state.yaml
│       ├── grasp_joint_state.yaml
│       └── lift_joint_state.yaml
├── fr3_vision_sorting/
│   ├── __init__.py
│   ├── fixed_pose_demo.py
│   ├── gripper_control.py
│   └── fixed_grasp_demo.py
├── package.xml
└── setup.py
```

## 1. Start the real FR3 and MoveIt

Enter the Docker container and source the workspaces:

```bash
docker exec -it fr3_vision_sorting bash

source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash
source /workspace/ros2_ws/install/setup.bash
```

Launch the official FR3 MoveIt configuration:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=172.16.0.2 \
  use_fake_hardware:=false
```

Keep this terminal running. In another container terminal, verify:

```bash
ros2 control list_controllers
ros2 action info /move_action
ros2 action list -t | grep -E "gripper|grasp"
```

Required action servers:

```text
/move_action
/franka_gripper/move
/franka_gripper/grasp
```

The `fr3_arm_controller` must be `active` before real motion.

## 2. Create the Stage 3 configuration directory

```bash
cd /workspace/ros2_ws/src/fr3_vision_sorting

mkdir -p config/fixed_grasp
```

If the four pose files already exist directly under `config/`, move them:

```bash
mv config/home_joint_state.yaml \
   config/pre_grasp_joint_state.yaml \
   config/grasp_joint_state.yaml \
   config/lift_joint_state.yaml \
   config/fixed_grasp/
```

Run `mv` only when those source files already exist. Otherwise, record them
directly into `config/fixed_grasp/` using the commands below.

## 3. Confirm the joint-state topic

```bash
ros2 topic list | grep joint_states
ros2 topic hz /joint_states
```

Use the topic that is actively publishing. The following examples assume
`/joint_states`.

## 4. Record `HOME`

Move the robot to the validated safe HOME pose. Wait until all motion stops,
then record one JointState message:

```bash
ros2 topic echo /joint_states --once \
  > /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_grasp/home_joint_state.yaml
```

Inspect the saved file:

```bash
cat /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_grasp/home_joint_state.yaml
```

## 5. Record `PRE_GRASP`

Move the gripper to a safe pose approximately 10–15 cm above the fixed object.
Keep the intended grasp orientation and verify table clearance.

```bash
ros2 topic echo /joint_states --once \
  > /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_grasp/pre_grasp_joint_state.yaml
```

Inspect it:

```bash
cat /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_grasp/pre_grasp_joint_state.yaml
```

## 6. Record `GRASP`

Open the gripper and descend slowly from PRE_GRASP until the fingers surround
the object at the intended contact height.

The GRASP pose must satisfy:

- the object is centered between the fingers;
- the fingers remain open while recording;
- the gripper does not touch the table;
- the fingers have enough object overlap for a stable grasp;
- the robot is not near a joint limit or singular configuration.

Record it:

```bash
ros2 topic echo /joint_states --once \
  > /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_grasp/grasp_joint_state.yaml
```

Inspect it:

```bash
cat /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_grasp/grasp_joint_state.yaml
```

## 7. Record `LIFT`

From GRASP, move vertically upward by approximately 10–15 cm while keeping the
same end-effector orientation.

```bash
ros2 topic echo /joint_states --once \
  > /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_grasp/lift_joint_state.yaml
```

Inspect it:

```bash
cat /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_grasp/lift_joint_state.yaml
```

## 8. Validate all saved poses

```bash
find \
  /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_grasp \
  -maxdepth 1 \
  -type f \
  -name "*.yaml" \
  -print
```

Expected files:

```text
home_joint_state.yaml
pre_grasp_joint_state.yaml
grasp_joint_state.yaml
lift_joint_state.yaml
```

The JointState message may contain seven FR3 arm joints and two gripper joints.
`fixed_grasp_demo.py` must select and reorder only:

```text
fr3_joint1 through fr3_joint7
```

Do not treat an arbitrary gripper joint value in this YAML as the measured
object width.

`ros2 topic echo` may add a YAML document separator (`---`). Therefore, load
the files with `yaml.safe_load_all()` and ignore empty documents:

```python
with path.open("r", encoding="utf-8") as stream:
    documents = [
        document
        for document in yaml.safe_load_all(stream)
        if isinstance(document, dict)
    ]

if not documents:
    raise ValueError(f"No valid JointState found in {path}")

data = documents[-1]
```

## 9. Configure `fixed_grasp_demo.py`

The node must load Stage 3 poses from the new directory:

```python
config_dir = (
    Path(get_package_share_directory("fr3_vision_sorting"))
    / "config"
    / "fixed_grasp"
)
```

The grasp parameters validated for the current test object were:

```python
def close_on_object(
    self,
    width: float = 0.039,
    speed: float = 0.02,
    force: float = 10.0,
) -> bool:
```

Use the same values in the Stage 3 sequence:

```python
if not demo.close_on_object(
    width=0.039,
    speed=0.02,
    force=10.0,
):
    return
```

The successful gripper tolerance was:

```python
goal.epsilon = GraspEpsilon(
    inner=0.005,
    outer=0.005,
)
```

These parameters are specific to the current object and contact geometry.

## 10. Update `setup.py`

Install the Stage 3 YAML directory:

```python
(
    os.path.join(
        "share",
        package_name,
        "config",
        "fixed_grasp",
    ),
    glob("config/fixed_grasp/*.yaml"),
),
```

Keep the Stage 3 executable:

```python
entry_points={
    "console_scripts": [
        "fixed_pose_demo = fr3_vision_sorting.fixed_pose_demo:main",
        "gripper_control = fr3_vision_sorting.gripper_control:main",
        "fixed_grasp_demo = fr3_vision_sorting.fixed_grasp_demo:main",
    ],
},
```

## 11. Build the package

```bash
cd /workspace/ros2_ws

source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash

colcon build --symlink-install \
  --packages-select fr3_vision_sorting

source install/setup.bash
```

Verify the executable and installed configuration:

```bash
ros2 pkg executables fr3_vision_sorting | grep fixed_grasp_demo

find \
  /workspace/ros2_ws/install/fr3_vision_sorting/share/fr3_vision_sorting/config/fixed_grasp \
  -type f \
  -name "*.yaml" \
  -print
```

## 12. Run Stage 3

```bash
ros2 run fr3_vision_sorting fixed_grasp_demo
```

Before each action, read the prompt, verify the physical scene, then enter
exactly:

```text
EXECUTE
```

The sequence must stop if grasp confirmation fails. Do not continue to LIFT
unless `/franka_gripper/grasp` returns `success: true`.

## Stage 3 experiment log

| Cycle | HOME | PRE_GRASP | GRASP | Grasp confirmed | LIFT | Return HOME | Notes |
|---:|:---:|:---:|:---:|:---:|:---:|:---:|---|
| 1 |  |  |  |  |  |  |  |
| 2 |  |  |  |  |  |  |  |
| 3 |  |  |  |  |  |  |  |
| 4 |  |  |  |  |  |  |  |
| 5 |  |  |  |  |  |  |  |
| 6 |  |  |  |  |  |  |  |
| 7 |  |  |  |  |  |  |  |
| 8 |  |  |  |  |  |  |  |
| 9 |  |  |  |  |  |  |  |
| 10 |  |  |  |  |  |  |  |

## Stage 3 success criteria

```text
10/10 fixed-position grasp cycles succeed
Grasp action returns success = true
Object remains stable during LIFT
No table, gripper, or self-collision occurs
No controller or Franka Desk error occurs
Robot safely returns to HOME
```

Do not add RealSense-based coordinates until this fixed-position baseline is
repeatable and safe.

---

[Development Process Index](README.md) · [Previous Stage](stage_02_fixed_pose_motion.md) · [Next Stage](stage_04_fixed_pick_place.md)
