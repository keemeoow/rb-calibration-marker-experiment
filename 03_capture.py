#!/usr/bin/env python3
"""03. Capture synchronized calibration images, depth, and robot state.

Input: Step 02 intrinsics, cameras, cube/board, robot, and optional waypoints.
Process: run legacy capture or the validated composite_rig_45_v1 pose plan.
Output: RGB-D, meta.json, recorded waypoints, and protocol completion manifest.
"""

def main() -> None:
    from capture_pipeline.capture import main as run
    run()


if __name__ == "__main__":
    main()
