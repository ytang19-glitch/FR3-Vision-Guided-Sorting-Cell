# Stage 7 — Vision-Guided Sorting

Final sequence:

```text
RealSense RGB-D
    ↓
Detect and classify object
    ↓
Calculate camera-frame XYZ
    ↓
Transform into fr3_link0
    ↓
Generate PRE_GRASP
    ↓
MoveIt planning
    ↓
Grasp object
    ↓
Place object in class-specific bin
```

Record:

- localization error;
- grasp success rate;
- sorting success rate;
- planning time;
- total cycle time.

---

---

[Development Process Index](README.md) · [Previous Stage](stage_06_calibration_tf2.md)
