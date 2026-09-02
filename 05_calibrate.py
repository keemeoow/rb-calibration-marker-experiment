#!/usr/bin/env python3
"""05. Run every executable Table 1 calibration condition.

Input: frozen observations, intrinsics, capture metadata, and robot FK.
Process: event-level split, shared initialization, fit, frame-prune, refit,
rollback, and held-out evaluation for A0-A5/B1-B3.
Output: table1_methods.json and authenticated shared-baseline artifacts.
"""

from calibration_pipeline.table1 import main


if __name__ == "__main__":
    main()
