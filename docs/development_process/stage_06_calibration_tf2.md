# Stage 6 — Calibration and TF2

For a fixed overhead RealSense, estimate:

```text
camera frame → fr3_link0
```

Then transform the detected point:

```text
Camera XYZ
    ↓
TF2 transformation
    ↓
FR3 base XYZ
```

Display the transformed position as an RViz marker. The marker must appear on the physical object before it is used as a motion target.

---

---

[Development Process Index](README.md) · [Previous Stage](stage_05_realsense_perception.md) · [Next Stage](stage_07_vision_guided_sorting.md)
