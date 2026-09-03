#!/usr/bin/env python3
"""Optional: evaluate fitted calibrations on frozen cross-target camera paths.

Input: Step 05 Table 1 JSON and the exact Step 04 observation manifest.
Process: reconstruct the frozen split and evaluate fixed-to-fixed and
gripper-to-fixed board/cube paths without external ground truth.
Output: cross_target_evaluation.json and cross_target_evaluation.csv.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from calibration_pipeline.cross_target import main


if __name__ == "__main__":
    main()
