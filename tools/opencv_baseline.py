#!/usr/bin/env python3
"""Optional: run the independent OpenCV fixed-camera relative-pose baseline.

Input: the Step 04 manifest and fixed camera intrinsics.
Process: solve target poses with OpenCV PnP without robot FK, hand-eye, or the
joint optimizer, then evaluate held-out fixed-camera relative consistency.
Output: OpenCV relative-baseline JSON, CSV, and Markdown artifacts.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from calibration_pipeline.opencv_relative_baseline import main


if __name__ == "__main__":
    main()
