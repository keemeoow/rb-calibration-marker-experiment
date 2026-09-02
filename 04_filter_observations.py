#!/usr/bin/env python3
"""04. Re-detect, filter, and freeze calibration corner observations.

Input: a captured calib_train directory and its fixed camera intrinsics.
Process: re-detect cube/board corners, classify quality, and hash source images.
Output: capture_filter manifest, CSV review files, retake list, and overlay.
"""

from calibration_pipeline.filter_observations import main


if __name__ == "__main__":
    main()
