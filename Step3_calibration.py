#!/usr/bin/env python3
"""Prepare the shared train-only baseline used by every Table 1 condition.

The former production calibration path duplicated initialization, pose-domain
refinement, and A3-specific optimization code. Step 3 is now a thin
compatibility entry point: it invokes ``calibration_pipeline.table1`` in
baseline-only mode, so Step3 and A0-A5/B1-B3 cannot drift.

Outputs in ``--out_dir``:

* ``shared_train_only_baseline.json`` — split, K/D contract, solver options,
  common reference state, and every row-specialized initial state;
* ``shared_board_free_fk_cube.json`` — train-only aligned FK cube artifact.

No final calibration transform is fitted here. The comparison runner consumes
the authenticated baseline and performs every executable row through the common
raw-corner pixel-reprojection backend.
"""

from __future__ import annotations

import sys

from calibration_pipeline.table1 import main as run_table1

def main() -> None:
    argv = list(sys.argv[1:])
    run_table1(argv, force_baseline_only=True)


if __name__ == "__main__":
    main()
