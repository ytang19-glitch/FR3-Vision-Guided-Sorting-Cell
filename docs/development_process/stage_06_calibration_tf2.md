# Stage 6 — Calibration and TF2

## Objective and confirmed setup

The D405 is fixed on a separate overhead support: this is an **eye-to-hand**
setup. Stage 5 provides annotated target detection and
`/object_point_camera`. Stage 6 finds the camera pose relative to the FR3
and validates object coordinates in `fr3_link0`.

Calibration estimates the transform. TF2 stores and applies it; TF2 does not
discover the calibration automatically. This page is a procedure and
implementation guide, not a record of completed calibration. No measured
transform has been supplied yet.

## 1. Verify Stage 5 before collecting calibration data

Keep the camera driver and one localizer running in separate sourced terminals:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash
```

Camera terminal (reuse it if already running):

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_sync:=true
```

Localizer terminal:

```bash
ros2 run fr3_vision_sorting camera_object_localizer
```

Inspection terminal:

```bash
ros2 topic info /object_point_camera --verbose
ros2 topic echo /object_point_camera
```

Use `rqt_image_view` to view
`/camera_object_localizer/annotated_image`.

- Move the cube by hand with the robot stationary. Verify that the contour
  follows the cube, not a separate red label on the cardboard.
- Cover other red regions for the initial experiment.
- Keep the cube stationary and record position variation. Earlier depth values
  varied by about 15 mm; if measured on a stationary target, investigate before
  accepting them for grasping.
- Confirm the depth window samples the cube surface rather than its edge or
  background. Temporal averaging cannot remove systematic depth bias.
- Confirm matching image dimensions, calibration profile and optical frame.
- The previously reported color distortion coefficients were nonzero. Raw
  pixels require distortion-aware deprojection; the simple pinhole inverse
  alone is not sufficient for precise raw-image coordinates. Use the matching
  camera model and distortion coefficients, or rectified pixels with matching
  rectified intrinsics. Depth alignment is not the same as image rectification.

The detected surface point is not automatically the cube center or gripper TCP
target.

## 2. Understand the transform

Use notation `T_A_B` for a transform mapping coordinates from frame B into A.

$$
\mathbf p_b = {}^b R_c\mathbf p_c + {}^b t_c
$$

$$
{}^b T_c =
\begin{bmatrix}
{}^b R_c & {}^b t_c\\
0 & 1
\end{bmatrix}
$$

| Symbol | Meaning |
|---|---|
| b | Robot base: `fr3_link0` |
| c | Color optical frame: `camera_color_optical_frame` |
| g | Chosen robot tool frame, for example `fr3_hand_tcp` |
| t | Calibration-target frame |
| `T_b_c` | Unknown fixed camera pose in the robot base |
| `T_b_g` | Robot tool pose from forward kinematics/TF |
| `T_c_t` | Target pose measured by the camera |
| `T_g_t` | Fixed target mounting pose relative to the tool |

For every paired measurement:

$$
{}^b T_g\,{}^g T_t = {}^b T_c\,{}^c T_t
$$

The target mounting transform can be unknown, but it must remain rigid while
collecting samples.

## 3. Prepare a calibration target

Use a flat printed checkerboard or ChArUco board fixed rigidly to the robot
end effector. Record its exact pattern geometry and measure the printed square
size in metres; disable printer scaling.

A red centroid alone supplies no target orientation and cannot replace the
full target pose required by this workflow.

Arrange the board so the fixed camera can see its pattern across several robot
orientations. Include the mounting fixture in clearance planning. Use the lab's
supervised robot operation procedure and keep stop controls accessible.

Before robot motion, implement and verify a target-pose detector. It should:

1. Detect ordered board corners.
2. Use known board coordinates and matching camera calibration to estimate
   `T_c_t` with a pose-estimation method such as PnP.
3. Account for distortion when using raw images.
4. Draw the detected corners and target axes.
5. Reject poor detections and report reprojection error.
6. Save the image timestamp, frame names, rotation and translation.

A correctly sized board should produce a physically plausible distance and
axes. Resolve board origin, axis direction and pose ambiguity before collection.
This detector is additional code; `camera_object_localizer.py` does not yet
provide full board poses.

## 4. Collect paired camera and robot poses

First verify the chosen tool frame:

```bash
ros2 run tf2_ros tf2_echo fr3_link0 fr3_hand_tcp
```

This displays the tool pose relative to the base. It is a diagnostic command,
not an automatic paired-data recorder.

For each sample:

1. Plan and execute a supervised, collision-checked move to a board-visible pose.
2. Wait until the arm and board are stationary.
3. Capture a valid `T_c_t` and the corresponding `T_b_g` using the image
   timestamp for the TF lookup.
4. Save both poses together, with timestamp and detection quality.
5. Repeat with varied positions and rotations about different axes.

Aim initially for roughly 15–25 useful samples, plus several held-out poses.
The number is a practical starting point, not an accuracy guarantee. Repeated
translations with identical orientation are inadequate for this hand-eye
workflow. Avoid nearly identical poses.

Use metres throughout and record quaternion ordering explicitly as ROS
`[x, y, z, w]`. Do not pair measurements taken at different robot poses.

## 5. Solve and check the calibration

One OpenCV route is `calibrateHandEye` adapted for eye-to-hand:
invert each robot `T_b_g` into `T_g_b`, retain each camera `T_c_t`,
and pass those arrays to the solver. Under this frame substitution, the
returned transform is `T_b_c`.

Illustrative offline solver code (requires collected arrays; does not move the robot):

```python
import cv2
import numpy as np

# T_b_g_samples and T_c_t_samples:
# corresponding lists of measured 4x4 matrices.
T_g_b_samples = [np.linalg.inv(T) for T in T_b_g_samples]

R_b_c, t_b_c = cv2.calibrateHandEye(
    [T[:3, :3] for T in T_g_b_samples],
    [T[:3, 3:4] for T in T_g_b_samples],
    [T[:3, :3] for T in T_c_t_samples],
    [T[:3, 3:4] for T in T_c_t_samples],
    method=cv2.CALIB_HAND_EYE_PARK,
)

T_b_c = np.eye(4)
T_b_c[:3, :3] = R_b_c
T_b_c[:3, 3] = np.asarray(t_b_c).reshape(3)

# These should agree across samples: the board mounting is rigid.
mount_estimates = [
    np.linalg.inv(T_b_g) @ T_b_c @ T_c_t
    for T_b_g, T_c_t in zip(T_b_g_samples, T_c_t_samples)
]
```

Check that the result is finite, the rotation is orthonormal with determinant
near +1, and the camera pose is physically plausible. Evaluate variation of
the mounting estimates on both training and held-out poses. A solver result
alone is not proof of accuracy.

Save the result with frame names, units, image profile, board dimensions and
validation residuals. Do not insert guessed translation or identity rotation
as a substitute for calibration.

See the [OpenCV calibration API](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
for transform conventions and eye-to-hand use.

## 6. Add the calibrated camera to the TF tree

Inspect the existing tree before adding a broadcaster. The RealSense driver
typically already publishes transforms from its camera root to the optical
frames. Each child frame must have one parent.

If `camera_color_optical_frame` already has a driver-published parent, do
not give it a second parent. Instead connect the camera root (verify its
actual name) to `fr3_link0`.

Let r denote that camera root. Obtain the existing `T_r_c` from TF, then:

$$
{}^b T_r = {}^b T_c ({}^r T_c)^{-1}
$$

Publish `T_b_r` as the fixed root connection and keep the driver's internal
camera transforms. Ensure no other broadcaster already supplies that root
connection.

Use measured translation and a normalized quaternion in a
`StaticTransformBroadcaster` or a launch-configured
`static_transform_publisher`. Do not launch a zero-filled example transform.

Verify the resulting composed transform:

```bash
ros2 run tf2_ros tf2_echo fr3_link0 camera_color_optical_frame
```

It should match the solved `T_b_c` and remain fixed while the robot moves.
See the [ROS 2 Jazzy static-transform tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html).

## 7. Transform object points into the base frame

Implement `object_tf_transformer.py` with a TF buffer/listener and a
subscription to `/object_point_camera`. Publish transformed points on
`/object_point_base`.

Core callback operation (inside a configured node):

```python
import tf2_geometry_msgs  # Registers PointStamped conversion with tf2.
from rclpy.duration import Duration
from tf2_ros import TransformException

try:
    point_base = self.tf_buffer.transform(
        point_camera,
        "fr3_link0",
        timeout=Duration(seconds=0.2),
    )
except TransformException:
    return  # Withhold output if the transform is unavailable.

self.point_base_publisher.publish(point_base)
```

Reject nonfinite points and stale input timestamps before this operation.
Preserve the measurement timestamp. Use a listener/executor configuration
that can receive TF while callbacks wait, or use a nonblocking lookup.

**Changing only `header.frame_id` is not a coordinate transformation.**

This is an implementation outline: the transformer needs to be created and
registered before a `ros2 run` command for it will work.

## 8. Validate without robot motion

Keep the robot stationary. View the transformed output:

```bash
ros2 topic echo /object_point_base
```

In RViz, set the Fixed Frame to `fr3_link0` and add a PointStamped display
for `/object_point_base`, or publish a visualization Marker and use a Marker
display.

Test several independently known target positions across the intended pickup
workspace, including positions not used for calibration. Compare the same
physical reference point: target surface, board corner or marker origin.
A visual overlap in RViz is useful but not a substitute for independent
measurements.

| Test position | Reference base XYZ (m) | Estimated base XYZ (m) | Error (mm) | Stationary variation (mm) |
|---|---|---|---|---|
| Center | Record | Record | Calculate | Measure |
| Left | Record | Record | Calculate | Measure |
| Right | Record | Record | Calculate | Measure |
| Near edge | Record | Record | Calculate | Measure |
| Far edge | Record | Record | Calculate | Measure |

Choose acceptance limits from the actual gripper clearance and task tolerance;
there is no universal 5 mm threshold. Validate the complete RGB-depth path too:
a good board-based calibration does not automatically fix depth bias.

Removing the cube must stop fresh detections. Any downstream controller must
expire old points rather than act on the last value indefinitely.

## 9. Generate pickup poses only after validation

Use the validated base-frame point to derive PRE_GRASP, GRASP and LIFT.
Account for cube dimensions, detected surface location, tool geometry and the
chosen TCP. Keep the validated tool orientation for the same frames.

Do not equate the detected top-surface Z with the grasp TCP Z. Define and verify
the grasp offset using the physical setup.

Plan with the table, fixtures and obstacles in the planning scene. Test only
PRE_GRASP first: review the plan, execute a supervised approach and verify
alignment above the cube. Plan the descent and retreat appropriately; saved
joint endpoints do not guarantee a straight vertical path.

## 10. First complete vision-guided cycle

Once localization and approach are validated:

1. Acquire a fresh, stable cube detection.
2. Transform to the base frame and check workspace limits.
3. Plan and approach PRE_GRASP with operator confirmation.
4. Open the gripper with verified clearance and descend to GRASP.
5. Close using the validated grasp parameters.
6. Confirm the grasp before lifting; stop the sequence on failure.
7. Lift and follow the saved PRE_BIN and BIN poses.
8. Release at the fixed labeled destination and retreat via POST_BIN.
9. Return HOME when the path is clear.

Keep the placement location fixed initially. Vision changes only the pickup
location. Moving the camera invalidates its fixed calibration; moving the
destination invalidates its saved placement poses.

## Immediate next lab task

Prepare a rigid, accurately measured calibration board and verify that the
camera can detect its corners and estimate its pose. Then build the paired
pose recorder before collecting robot poses. Keep the existing fixed
pick-and-place baseline available while developing these perception tools.

---

[Development Process Index](README.md) · [Previous Stage](stage_05_realsense_perception.md) · [Next Stage](stage_07_vision_guided_sorting.md)
