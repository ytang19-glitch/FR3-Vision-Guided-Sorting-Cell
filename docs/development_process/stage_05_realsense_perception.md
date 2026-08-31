# Stage 5 — RealSense Perception

After fixed-position pick and place works:

```text
RGB image
    ↓
OpenCV or YOLO detection
    ↓
Object class
    ↓
Center pixel (u,v)
    ↓
Aligned depth
    ↓
Camera-frame XYZ
```

At this stage, only calculate and visualize the object position. Do not immediately command the real FR3.

---

---

[Development Process Index](README.md) · [Previous Stage](stage_04_fixed_pick_place.md) · [Next Stage](stage_06_calibration_tf2.md)
