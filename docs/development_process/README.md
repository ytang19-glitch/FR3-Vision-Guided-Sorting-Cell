# FR3 Vision-Guided Sorting Cell — Development Process

This guide teaches students how to build the FR3 sorting experiment incrementally, from the official robot model to autonomous vision-guided sorting.

## Development Roadmap

```text
Stage 1: Official FR3 model and fake hardware
        ↓
Stage 2: HOME ↔ PRE_GRASP fixed-pose motion
        ↓
Stage 3: Fixed-position grasping
        ↓
Stage 4: Fixed-position pick and place
        ↓
Stage 5: RealSense RGB-D perception
        ↓
Stage 6: Camera calibration and TF2
        ↓
Stage 7: Vision-guided sorting
```

## Stage Guides

| Stage | Guide | Main result |
|---:|---|---|
| 1 | [Official FR3 model](stage_01_official_fr3.md) | Verified URDF, TF, controllers and fake hardware |
| 2 | [Repeatable fixed-pose motion](stage_02_fixed_pose_motion.md) | `HOME → PRE_GRASP → HOME` |
| 3 | [Fixed-position grasping](stage_03_fixed_grasping.md) | Grasp and lift one known object |
| 4 | [Fixed-position pick and place](stage_04_fixed_pick_place.md) | Move one object to a known bin |
| 5 | [RealSense perception](stage_05_realsense_perception.md) | Detect object and recover camera-frame XYZ |
| 6 | [Calibration and TF2](stage_06_calibration_tf2.md) | Transform camera XYZ into `fr3_link0` |
| 7 | [Vision-guided sorting](stage_07_vision_guided_sorting.md) | Autonomous class-based sorting |

Read the [Safety Principle](SAFETY.md) before operating real hardware.

## Project Structure

The project should have the following structure:

```text
fr3_vision_sorting/
├── Dockerfile
├── compose.yaml
├── start_container.sh
├── README.md
├── docs/
│   └── development_process/
│       ├── README.md
│       └── stage_01_official_fr3.md
└── ros2_ws/
    └── src/
        └── fr3_vision_sorting/
            ├── package.xml
            ├── setup.py
            ├── setup.cfg
            ├── resource/
            │   └── fr3_vision_sorting
            ├── config/
            │   ├── home_joint_state.yaml
            │   └── pre_grasp_joint_state.yaml
            └── fr3_vision_sorting/
                ├── __init__.py
                └── fixed_pose_demo.py
```

The official Franka packages are installed separately:

```text
/opt/franka_ros2_ws
```

Do not modify the official Franka workspace. Store student code in:

```text
/workspace/ros2_ws
```

---
