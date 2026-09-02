#!/usr/bin/env python3
"""02. Calibrate RealSense color intrinsics with ChArUco observations.

Input: Step 01 intrinsics, connected cameras, and the configured ChArUco board.
Process: collect coverage-controlled views and run OpenCV camera calibration.
Output: updated cam*.npz plus factory_backup/ and optional capture images.
"""

def main() -> None:
    from capture_pipeline.calibrate_intrinsics import main as run
    run()


if __name__ == "__main__":
    main()
