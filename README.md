# Unified Multi-Fixed-Camera + Robot Hand-Eye + AprilTag Cube Calibration

## Overview

This pipeline calibrates a system consisting of:
- **N fixed cameras** (RealSense, mounted around workspace)
- **1 robot gripper camera** (RealSense, eye-in-hand)
- **AprilTag cube** (calibration target)

### Coordinate Frames
```
Base (Robot)
 ├── Gripper (EE) ── gTc ──> Gripper Camera
 │                              │
 │                         sees AprilTag Cube
 │                              │
 ├── bTo (cube in base) <──────-┘
 │
 └── bTfi (fixed cam i in base)
      ├── Fixed Cam 0 (ref)
      ├── Fixed Cam 1
      ├── Fixed Cam 2
      └── Fixed Cam 3
```

### Pipeline Steps

| Step | Script | Description |
|------|--------|-------------|
| 1 | `Step1_dump_all_intrinsics.py` | Dump factory intrinsics for all cameras (fixed + gripper) |
| 1b | `Step1b_charuco_intrinsics.py` | Refine intrinsics from a ChArUco capture (overwrites `intrinsics/cam*.npz`, keeps `factory_backup/`) |
| 2 | `Step2_capture.py` | Capture AprilTag cube from all cameras + record robot joints |
| 3 | `Step3_calibration.py` | Compute: fixed cam extrinsics, hand-eye (gTc), fixed-to-base transforms |
| 4 | `Step4_verify.py` | Verify calibration accuracy, render 3D scene |
| 5 | `Step5_export_reports.py` | Export summary reports from a finished calibration |

### Usage
```bash
# Step 1: Dump intrinsics (connect ALL cameras)
python Step1_dump_all_intrinsics.py --out_dir ./intrinsics

# Step 1b (optional): refine intrinsics with a ChArUco board
python Step1b_charuco_intrinsics.py --intr_dir ./intrinsics --save_images

# Step 2: Capture (robot moves cube, all cameras see it)
# --root_folder is intentionally omitted: the next data/sessionNN/calib_train
# directory is allocated automatically and is never overwritten.
python Step2_capture.py \
  --data_root ./data \
  --intrinsics_dir ./intrinsics \
  --use_robot --robot_ip 192.168.0.23 --robot_port 12348 \
  --show

# Step 3: Calibrate everything (replace sessionNN with the path printed by Step 2)
python Step3_calibration.py \
  --root_folder ./data/sessionNN/calib_train \
  --intrinsics_dir ./intrinsics \
  --gripper_cam_idx 0 \
  --ref_fixed_cam_idx 1

# Step 4: Verify
python Step4_verify.py \
  --root_folder ./data/sessionNN/calib_train \
  --intrinsics_dir ./intrinsics

# Step 5: Export reports
python Step5_export_reports.py \
  --root_folder ./data/sessionNN/calib_train \
  --intrinsics_dir ./intrinsics
```

Depth is always captured and used; there is no `--save_depth` flag.

### Experiments

Paper experiments (`CP_*.py`) are documented separately in
[CP_EXPERIMENTS_README.md](CP_EXPERIMENTS_README.md); synthetic counterparts live in
[Simul_test/](Simul_test/). The canonical results table is
[Calibration_Experiment_table.md](Calibration_Experiment_table.md).

[PRESENTATION_PROMPT.md](PRESENTATION_PROMPT.md) is the seminar-slide generation prompt:
it fixes which claims the results support, which they do not, and the reporting rules
(FK-proxy labelling, px/mm separation, common-component comparisons).
