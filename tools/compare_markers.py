#!/usr/bin/env python3
"""Optional: compare board-only, cube-only, and combined marker systems.

Input: the Step 04 manifest, intrinsics, capture metadata, and robot FK.
Process: fit each marker modality on the same frozen train/test protocol.
Output: marker_system_end_to_end.json and marker_system_end_to_end.csv.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from calibration_pipeline.marker_system import main


if __name__ == "__main__":
    main()
