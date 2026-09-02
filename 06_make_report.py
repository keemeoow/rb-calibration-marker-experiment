#!/usr/bin/env python3
"""06. Build a detailed calibration report from the Step 05 result only.

Input: table1_methods.json produced by 05_calibrate.py.
Process: summarize convergence, reprojection, frame-prune decisions, and every
final transform without running another optimizer or selecting by held-out data.
Output: calibration_summary.csv, calibration_matrices.json, and
CALIBRATION_RESULTS.md in the Table 1 output directory.
"""

from calibration_pipeline.report import main


if __name__ == "__main__":
    main()
