#!/usr/bin/env python3
"""03. Capture synchronized calibration images, depth, and robot FK.

Input: Step 02 intrinsics, cameras, cube/board, robot, and optional waypoints.
Process: synchronize streams, apply capture gates, and record pose provenance.
Output: data/sessionNN/calib_train images and meta.json.
"""

def main() -> None:
    from capture_pipeline.capture import main as run
    run()


if __name__ == "__main__":
    main()
