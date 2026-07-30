#!/usr/bin/env python3
"""Materialize the canonical board-free FK-cube artifact and comparison."""
from __future__ import annotations

import argparse
import json
import os

from CP_ablation_7row import _jsonable, prepare_ablation_data
from CP_ablation_schema import validate_fk_alignment_artifact


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root_folder", default="data/session")
    parser.add_argument("--intrinsics_dir", default="intrinsics")
    parser.add_argument("--calib_dir", default="data/session/calib_out")
    parser.add_argument("--out_dir", default="CP_result/fk_cube_artifact")
    parser.add_argument("--test_fraction", type=float, default=0.2)
    parser.add_argument("--split_seed", type=int, default=20260729)
    parser.add_argument("--min_train_eih_cube_events", type=int, default=3)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    prepared = prepare_ablation_data(args)
    artifact = prepared.alignment_artifact
    comparison = prepared.alignment_comparison
    validate_fk_alignment_artifact(artifact)
    os.makedirs(args.out_dir, exist_ok=True)
    artifact_path = os.path.join(args.out_dir, "shared_board_free_fk_cube.json")
    comparison_path = os.path.join(
        args.out_dir, "supplementary_board_free_vs_board_derived_fk.json")
    with open(artifact_path, "w") as handle:
        json.dump(_jsonable(artifact), handle, indent=2)
    with open(comparison_path, "w") as handle:
        json.dump(_jsonable(comparison), handle, indent=2)
    nominal = artifact["runs"][0]
    lines = [
        "# Board-free FK-cube artifact",
        "",
        "Canonical consumers: **B1, A3, B2**. Board and held-out information: **not used**.",
        "",
        f"- SHA-256: `{artifact['artifact_sha256']}`",
        f"- Train eih cube support: {artifact['source_counts']['observations']} observations / "
        f"{artifact['source_counts']['corners']} corners / {artifact['source_counts']['sets']} sets",
        f"- Solver: status {nominal['status']}, nfev {nominal['nfev']}, "
        f"reprojection {nominal['train_reprojection_rmse_px']:.6f} px",
        f"- Jacobian: rank {nominal['jacobian']['rank']}/{nominal['jacobian']['n_params']}, "
        f"condition {nominal['jacobian']['jacobian_condition_number']:.3f}",
        f"- Board-derived delta difference (supplementary): "
        f"{comparison['canonical_vs_board_derived_fk_delta_translation_mm']:.3f} mm / "
        f"{comparison['canonical_vs_board_derived_fk_delta_rotation_deg']:.3f}°",
        "",
        "The supplementary comparison is not consumed by any ablation row.",
        "",
    ]
    with open(os.path.join(args.out_dir, "README.md"), "w") as handle:
        handle.write("\n".join(lines))
    print(f"[SAVE] {artifact_path}")
    print(f"[SAVE] {comparison_path}")
    print(f"[SHA256] {artifact['artifact_sha256']}")


if __name__ == "__main__":
    main()
