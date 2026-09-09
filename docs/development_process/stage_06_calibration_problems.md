# Stage 6 — Camera-to-FR3 Calibration Problems and Debugging Log

This document records the practical issues encountered while developing **Stage 6: Intel RealSense D405 camera-to-Franka FR3 calibration**.

The setup is an **eye-to-hand** configuration: the D405 is fixed above the workspace and the FR3 moves independently. The goal is to determine the fixed transform from `camera_color_optical_frame` to `fr3_link0` so that camera-frame object coordinates can be converted into robot-base coordinates.

## Current pipeline

```text
D405
  ↓
realsense2_camera
  ↓
/camera/camera/color/image_raw
  ↓
cv_bridge
  ↓
OpenCV / ArUco
  ↓
ChArUco board detection
  ↓
board pose in camera frame
  ↓
camera-to-FR3 extrinsic calibration
  ↓
T_base_camera
  ↓
TF2
  ↓
P_camera → P_base
```

## Problems encountered

| # | Problem | Observed symptom | Cause / lesson | Status |
|---|---|---|---|---|
| 1 | ChArUco board parameters were uncertain | Confusion between `22/16 mm`, `20/15 mm`, and other board dimensions | Detector/calibration parameters must correspond to the actual printed board | Needs final measurement |
| 2 | PDF dimensions differ from measured print | Calib.io PDF specifies 22 mm checker / 16 mm marker, while the physical print was measured around 20 mm / 15 mm | Printer scaling can change physical dimensions; use measured dimensions for metric pose estimation | Important before pose calibration |
| 3 | `squareLength` vs `markerLength` confusion | Initially unclear whether these described the whole board | `squareLength` is one checker-square side; `markerLength` is one ArUco marker side | Resolved |
| 4 | ArUco dictionary mismatch suspected | `0/17 markers detected` raised concern that Calib.io `DICT_4X4` did not match OpenCV | Current detector uses `cv2.aruco.DICT_4X4_50`; successful partial detections show the marker family is being decoded | Largely resolved |
| 5 | `0/17 markers detected` | Camera could visibly see the board but OpenCV detected no markers | Board was very small in the D405 image, so individual markers had insufficient image resolution | Main cause identified |
| 6 | Only partial marker detection | After moving the board closer, detection increased to approximately `7/17`, `8/17`, and `9/17` | Detection works, but distance, angle, blur, exposure, focus, and marker pixel size still affect reliability | In progress |
| 7 | OpenCV API incompatibility | `AttributeError: module 'cv2.aruco' has no attribute 'ArucoDetector'` | Installed OpenCV is 4.6.0 and does not expose the newer `ArucoDetector` API in this environment | Resolved |
| 8 | Need OpenCV 4.6-compatible ArUco API | New detector example could not run | Use `cv2.aruco.detectMarkers()` and `cv2.aruco.DetectorParameters_create()` | Resolved |
| 9 | Direct camera access failed | `VIDEOIO(V4L2:/dev/video0): can't open camera by index` and `ERROR: Cannot open camera` | Docker/container did not expose the D405 as `/dev/video0` for `cv2.VideoCapture(0)` | Resolved by using ROS 2 |
| 10 | Docker camera-access confusion | OpenCV direct access failed although RealSense ROS driver worked | In this project it is cleaner to consume the existing ROS 2 camera stream rather than opening V4L2 directly | Resolved |
| 11 | Need to bridge ROS images to OpenCV | Detector originally expected `VideoCapture(0)` | Subscribe to `/camera/camera/color/image_raw` and convert with `cv_bridge` | Resolved |
| 12 | Need to distinguish camera failure from detector failure | Initially unclear whether `0/17` meant the D405 was not working | Verify topics with `ros2 topic list` and frame rate with `ros2 topic hz` before debugging ArUco | Resolved |
| 13 | Board too far from camera | ChArUco pattern occupied only a small part of the image | Move board closer during detector testing so individual markers contain enough pixels | Confirmed |
| 14 | Board orientation requirements unclear | Uncertainty about whether the board must be horizontal or have a specific orientation | Board need not remain in one orientation; calibration benefits from multiple visible poses and rotational diversity | Collection requirement |
| 15 | ROS 2 CLI syntax errors | `ros2: error: unrecognized arguments` while inspecting calibration topics | Topic names and `ros2 topic echo` syntax must be entered correctly | Needs cleanup |
| 16 | `/calibration_board/pose` inspection issue | Attempts to inspect the board-pose topic produced CLI errors | Verify exact topic name/type and then use `ros2 topic echo <topic>` | In progress |
| 17 | Calibration output topics need verification | `/calibration_board/pose`, annotated image, and reprojection topics were inspected during debugging | A topic name appearing is not enough; verify publisher, message type, timestamp, and live data | In progress |
| 18 | ArUco detection confused with ChArUco corner detection | Initial debugging focused on number of markers only | ArUco marker detection is the first stage; ChArUco corner interpolation and pose estimation come afterward | Next step |
| 19 | Physical marker dimensions incorrectly suspected as cause of `0/17` | Concern that 20/15 vs 22/16 caused zero marker detection | Basic `detectMarkers()` decodes image patterns and does not require physical marker dimensions | Resolved |
| 20 | Physical dimensions still matter for pose scale | Printed dimensions may differ from PDF | Wrong square/marker dimensions create scale error in estimated metric pose and therefore camera-to-FR3 calibration | Critical |
| 21 | Camera intrinsics must match the image | Board pose requires camera matrix and distortion coefficients | Pose estimation must use the D405 color-camera intrinsics corresponding to the actual image stream | To validate |
| 22 | Camera and robot use different coordinate frames | `P_camera` cannot directly be sent to the FR3 | Stage 6 must estimate `T_base_camera` to transform camera points into `fr3_link0` | Core Stage 6 task |
| 23 | TF frame availability can be transient | Earlier tests produced `Invalid frame ID fr3_link0` | TF publishers/startup order must be stable before synchronized calibration collection | Needs validation |
| 24 | Tool-frame availability issue | `fr3_hand_tcp` could temporarily appear unavailable and later resolve | Verify the actual tool frame and wait for the complete TF tree before collecting samples | Needs validation |
| 25 | Calibration geometry must remain consistent | Fixed-camera and moving-camera procedures can easily be mixed conceptually | This setup is eye-to-hand: D405 fixed, FR3 moving; the chosen board/tool relationship must be rigid and modeled consistently | Defined |
| 26 | Calibration result still needs independent validation | A plausible transform or RViz view is not proof of accuracy | Validate `T_base_camera` using reserved board poses, reprojection/pose residuals, and independent physical points before vision-guided motion | Final validation required |

## Key debugging result

The detector originally produced:

```text
Detected: 0/17 markers
```

After the ROS 2 image pipeline was confirmed and the board was moved closer to the D405, detection increased to values such as:

```text
Detected: 7/17 markers
Detected: 8/17 markers
Detected: 9/17 markers
```

This is an important distinction. Partial detection demonstrates that the following components are functioning:

```text
OpenCV 4.6              ✓
cv2.aruco                ✓
D405 image stream        ✓
ROS 2                    ✓
cv_bridge                ✓
4x4 ArUco decoding       ✓
printed markers          ✓
```

The immediate task is therefore no longer basic camera connectivity. It is to improve robust board detection and then proceed to ChArUco pose estimation.

## OpenCV 4.6 compatibility

The installed environment reports:

```text
OpenCV version: 4.6.0
```

The following newer API is not available:

```python
cv2.aruco.ArucoDetector(dictionary)
```

Use the OpenCV 4.6-compatible interface instead:

```python
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

corners, ids, rejected = cv2.aruco.detectMarkers(
    frame,
    dictionary,
    parameters=parameters,
)
```

## Camera-access lesson

Direct access through:

```python
cv2.VideoCapture(0)
```

failed inside the container with:

```text
VIDEOIO(V4L2:/dev/video0): can't open camera by index
ERROR: Cannot open camera
```

The working approach is:

```text
D405
 ↓
realsense2_camera
 ↓
/camera/camera/color/image_raw
 ↓
cv_bridge
 ↓
OpenCV
```

This also keeps the calibration detector consistent with the rest of the ROS 2 vision pipeline.

## Board geometry

The Calib.io source board is labeled approximately as:

```text
5 × 7 squares
Checker size: 22 mm
Marker size: 16 mm
Dictionary: ArUco DICT_4X4
```

However, the physical print was measured at approximately:

```text
square side ≈ 20 mm
marker side ≈ 15 mm
```

The physical dimensions do **not** determine whether `detectMarkers()` can decode the marker pattern. They **do** determine the metric scale of later pose estimation. Therefore the final board dimensions should be carefully re-measured before computing the camera-to-robot transform.

## Next debugging sequence

```text
Current: 7–9 / 17 markers
          ↓
improve distance / focus / angle / exposure
          ↓
stable high marker detection
          ↓
interpolate ChArUco corners
          ↓
estimate board pose T_camera_board
          ↓
collect synchronized camera + FR3 poses
          ↓
solve eye-to-hand calibration
          ↓
obtain T_base_camera
          ↓
publish/verify TF2 transform
          ↓
P_camera → P_base
          ↓
validate with independent physical points
          ↓
vision-guided PRE_GRASP
```

## Success criteria before moving to robot motion

Do not use the calibration for autonomous picking until:

1. The board is detected reliably over a useful range of poses.
2. ChArUco corners are correctly associated with known metric coordinates.
3. D405 color intrinsics and distortion coefficients are verified.
4. Camera and robot pose samples are synchronized.
5. The dataset contains meaningful position and orientation diversity.
6. `T_base_camera` is stable and physically plausible.
7. Reserved validation poses produce acceptable residual errors.
8. Known physical points transformed from camera coordinates agree with measurements in `fr3_link0`.
9. TF2 consistently resolves `fr3_link0 ↔ camera_color_optical_frame`.
10. Only then is the transformed target used to generate a supervised PRE_GRASP pose.

---

Related Stage 6 documentation:

- `stage_06_calibration_tf2.md`
- `stage_06_calibration_charuco_board_camera_to_robot.md`
- `stage_06_calibration_ChArUco.pdf`
