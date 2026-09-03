#!/usr/bin/env python3
"""01. Export factory intrinsics and depth scale from every RealSense.

Input: connected RealSense devices and the requested stream resolution/FPS.
Process: enumerate stable serial-to-camera IDs and read color/depth intrinsics.
Output: intrinsics/device_map.json, depth_scales.json, and cam*.npz files.
"""

def main() -> None:
    from capture_pipeline.export_intrinsics import main as run
    run()


if __name__ == "__main__":
    main()
