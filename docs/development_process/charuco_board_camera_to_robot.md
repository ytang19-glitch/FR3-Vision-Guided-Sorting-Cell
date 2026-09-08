# ChArUco Board: Connecting Camera Coordinates to FR3 Base Coordinates

## Purpose

The RealSense D405 detects an object in the **camera coordinate frame**, but the FR3 robot plans and moves in its **base coordinate frame** (`fr3_link0`).

The purpose of the ChArUco board is to provide a known geometric reference that both the camera and robot calibration procedure can use to estimate the rigid transform between these two coordinate systems.

---

## 1. Camera Coordinate System

The D405 localizer produces a 3D object point in the camera frame:

```math
P_{camera}=
\begin{bmatrix}
X_c\\
Y_c\\
Z_c
\end{bmatrix}
```

For example:

```text
/object_point_camera
x: 0.08 m
y: 0.04 m
z: 0.62 m
```

This tells us where the cube is relative to the camera.

However, the FR3 cannot directly use this point for motion planning because it does not know where the camera frame is relative to `fr3_link0`.

---

## 2. Robot Base Coordinate System

The FR3 needs the same object position expressed in its base frame:

```math
P_{base}=
\begin{bmatrix}
X_b\\
Y_b\\
Z_b
\end{bmatrix}
```

This is the coordinate system used for robot motion commands and grasp poses.

---

## 3. Why Use a ChArUco Board?

A ChArUco board combines:

- checkerboard corners for accurate geometric localization;
- ArUco marker IDs so OpenCV knows which part of the board is visible;
- known physical dimensions such as square size and marker size.

Because the board geometry is known, OpenCV can estimate the board's full 6-DoF pose relative to the camera:

```math
{}^{camera}T_{board}
```

If the board pose relative to the FR3 base is also known or measured:

```math
{}^{base}T_{board}
```

then the camera-to-base transform can be calculated.

---

## 4. Camera-to-Robot Transformation

The quantity we ultimately need is:

```math
\boxed{{}^{base}T_{camera}}
```

This transformation describes the position and orientation of the camera relative to the FR3 base.

A homogeneous transformation has the form:

```math
{}^{base}T_{camera}=
\begin{bmatrix}
R & t\\
0 & 1
\end{bmatrix}
```

where:

- `R` is the 3×3 rotation matrix;
- `t` is the 3×1 translation vector.

---

## 5. Converting a Detected Cube Position

Represent the camera-frame point in homogeneous coordinates:

```math
\tilde{P}_{camera}=
\begin{bmatrix}
X_c\\
Y_c\\
Z_c\\
1
\end{bmatrix}
```

Then transform it into the FR3 base frame:

```math
\tilde{P}_{base}
=
{}^{base}T_{camera}
\tilde{P}_{camera}
```

or equivalently:

```math
P_{base}=R P_{camera}+t
```

The result is the cube position that the FR3 can use for planning a pre-grasp and grasp pose.

---

## 6. Full Vision-Guided Pick Pipeline

```text
RealSense D405 sees cube
          |
          v
Detect cube pixel center (u, v)
          |
          v
Read aligned depth Zc
          |
          v
Convert pixel + depth to camera XYZ
          |
          v
P_camera = [Xc, Yc, Zc]
          |
          |  apply  ^base T_camera
          v
P_base = [Xb, Yb, Zb]
          |
          v
Create PRE_GRASP pose
          |
          v
Move FR3 above cube
          |
          v
Descend -> close gripper -> lift
```

The key transition is:

```math
\boxed{P_{camera}\rightarrow{}^{base}T_{camera}\rightarrow P_{base}}
```

---

## 7. Where the ChArUco Board Fits

The ChArUco board is mainly a **calibration tool**.

It does not need to remain in the workspace during normal pick-and-place operation.

Typical workflow:

```text
1. Fix the D405 camera rigidly.
2. Print and measure the ChArUco board.
3. Place the board in the FR3 workspace.
4. Detect the board with the D405.
5. Determine the board pose relative to fr3_link0.
6. Compute ^base T_camera.
7. Validate the transform at several known points.
8. Save/publish the transform in TF2.
9. Remove the ChArUco board.
10. Detect cubes and transform their positions into fr3_link0.
```

If the camera is moved afterward, the calibration must be repeated.

---

## 8. Board Used for This Project

Recommended starting board:

```text
ChArUco layout:      5 × 7 squares
Square length:       30 mm = 0.030 m
ArUco marker length: 22 mm = 0.022 m
Paper:               A4
Approx. board area:  150 × 210 mm
Dictionary:          DICT_4X4_50
```

The dimensions used in OpenCV must exactly match the physical printed board.

Example:

```python
squares_x = 5
squares_y = 7
square_length = 0.030
marker_length = 0.022
```

Always print at **100% / Actual Size**, then verify the square dimensions with a ruler or caliper.

---

## 9. Success Condition

Calibration is successful when a point measured by the D405 can be transformed into `fr3_link0` with sufficiently small error.

For several test points, compare:

```text
Camera detection -> transformed base XYZ
```

against:

```text
Known/measured FR3 base XYZ
```

Only after this validation should the FR3 use the transformed cube position for real motion.

---

## Key Idea

> The D405 tells us where the cube is relative to the camera. The ChArUco calibration tells us where the camera is relative to the FR3. Combining both lets the FR3 know where the cube is relative to `fr3_link0`.

```math
\boxed{
P_{camera}
\xrightarrow{{}^{base}T_{camera}}
P_{base}
\xrightarrow{\text{motion planning}}
\text{FR3 moves to cube}
}
```
