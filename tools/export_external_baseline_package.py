#!/usr/bin/env python3
"""Export a neutral frozen-observation package for external baselines.

This addresses the 8/3 feedback asking to run the same images through public
implementations.  The package does not execute COLMAP/MATLAB itself.  Instead,
it freezes the exact RGB image references, 2D image points, 3D object points,
intrinsics, and train/test split that an external adapter must consume.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = (
    ROOT / "data/session04/calib_out/capture_filter/"
    "Step2b_observation_manifest.json"
)
DEFAULT_TABLE1 = ROOT / "CP_result/session04/late_table1/table1_methods.json"
DEFAULT_OUT_DIR = ROOT / "CP_result/session04/external_baseline_package"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): jsonable(val) for key, val in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(item) for item in value]
    return value


def load_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def selected_observations(manifest: dict, policy: str, split: dict) -> list[dict]:
    train_events = set(map(int, split["train_events"]))
    test_events = set(map(int, split["test_events"]))
    eligible_events = train_events | test_events
    rows = []
    for record in manifest["observations"]:
        if not record.get("selected_by_policy", {}).get(policy, False):
            continue
        event_id = int(record["event_id"])
        if event_id not in eligible_events:
            continue
        split_name = "train" if event_id in train_events else "test"
        camera_id = int(record["camera_id"])
        rows.append({
            "observation_id": str(record["observation_id"]),
            "event_id": event_id,
            "set_idx": int(record["set_idx"]),
            "split": split_name,
            "camera_id": camera_id,
            "camera_role": "gripper" if camera_id == 2 else "fixed",
            "target": str(record["target"]),
            "capture_block": str(record.get("capture_block", "")),
            "cube_gripped": bool(record.get("cube_gripped", False)),
            "image_path": str(record["image_path"]),
            "corner_count": int(record["corner_count"]),
            "charuco_ids": record.get("charuco_ids", []),
            "marker_ids": record.get("marker_ids", []),
            "object_points": record["object_points"],
            "image_points": record["image_points"],
        })
    rows.sort(key=lambda row: (
        row["split"], row["event_id"], row["camera_id"], row["target"],
        row["observation_id"],
    ))
    return rows


def validate_sources(manifest: dict, rows: list[dict],
                     session_root: Path, intrinsics_dir: Path) -> dict:
    images = manifest["source"]["images"]
    checked_images = {}
    for row in rows:
        rel = row["image_path"]
        local_path = session_root / rel
        expected = str(images[rel]["sha256"])
        actual = sha256_file(local_path)
        if actual != expected:
            raise ValueError(f"image SHA mismatch: {local_path}")
        checked_images[rel] = {
            "repo_relative_path": str(local_path.relative_to(ROOT)),
            "sha256": actual,
        }

    checked_intrinsics = {}
    for camera_id, source in sorted(manifest["source"]["intrinsics"].items(),
                                    key=lambda item: int(item[0])):
        local_path = intrinsics_dir / f"cam{int(camera_id)}.npz"
        expected = str(source["sha256"])
        actual = sha256_file(local_path)
        if actual != expected:
            raise ValueError(f"intrinsics SHA mismatch: {local_path}")
        with np.load(local_path) as data:
            k_key = "K" if "K" in data.files else "color_K"
            d_key = "D" if "D" in data.files else "color_D"
            checked_intrinsics[str(camera_id)] = {
                "repo_relative_path": str(local_path.relative_to(ROOT)),
                "sha256": actual,
                "K_key": k_key,
                "D_key": d_key,
                "K": jsonable(np.asarray(data[k_key], dtype=float)),
                "D": jsonable(np.asarray(data[d_key], dtype=float).reshape(-1)),
            }
    return {
        "images": checked_images,
        "intrinsics": checked_intrinsics,
    }


def count_rows(rows: list[dict]) -> dict:
    counters = {
        "by_split": Counter(),
        "by_target": Counter(),
        "by_split_target": Counter(),
        "by_camera_role": Counter(),
        "by_camera": Counter(),
    }
    total_corners = 0
    for row in rows:
        corners = int(row["corner_count"])
        total_corners += corners
        counters["by_split"][row["split"]] += 1
        counters["by_target"][row["target"]] += 1
        counters["by_split_target"][f"{row['split']}:{row['target']}"] += 1
        counters["by_camera_role"][row["camera_role"]] += 1
        counters["by_camera"][str(row["camera_id"])] += 1
    return {
        "observations": len(rows),
        "corners": total_corners,
        **{name: dict(counter) for name, counter in counters.items()},
    }


def write_csv(rows: list[dict], path: Path) -> None:
    fieldnames = [
        "observation_id", "event_id", "set_idx", "split", "camera_id",
        "camera_role", "target", "capture_block", "cube_gripped",
        "image_path", "corner_count", "charuco_ids_json", "marker_ids_json",
        "object_points_json", "image_points_json",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({
                "observation_id": row["observation_id"],
                "event_id": row["event_id"],
                "set_idx": row["set_idx"],
                "split": row["split"],
                "camera_id": row["camera_id"],
                "camera_role": row["camera_role"],
                "target": row["target"],
                "capture_block": row["capture_block"],
                "cube_gripped": row["cube_gripped"],
                "image_path": row["image_path"],
                "corner_count": row["corner_count"],
                "charuco_ids_json": json.dumps(row["charuco_ids"]),
                "marker_ids_json": json.dumps(row["marker_ids"]),
                "object_points_json": json.dumps(row["object_points"]),
                "image_points_json": json.dumps(row["image_points"]),
            })


def write_markdown(package: dict, path: Path) -> None:
    counts = package["counts"]
    lines = [
        "# External Baseline Frozen-observation Package",
        "",
        "목적: 8/3 피드백 #7, 즉 같은 사진을 공개 구현에도 넣어 custom "
        "optimizer만의 문제인지 확인하라는 요구를 재현 가능한 입력 계약으로 "
        "분리한다.",
        "",
        "이 package는 외부 baseline을 실행한 결과가 아니라 **외부 baseline이 반드시 "
        "사용해야 하는 frozen input**이다. Detector를 다시 돌리지 않고, Step 04에서 "
        "동결한 2D image points와 3D object points를 그대로 사용한다.",
        "",
        "## 포함된 파일",
        "",
        "- `external_baseline_observations.csv`: observation 단위 frozen 2D/3D correspondences",
        "- `external_baseline_package.json`: split, source SHA-256, contract, counts",
        "- `external_baseline_intrinsics.json`: fixed camera intrinsics",
        "",
        "## 현재 support",
        "",
        f"- Observations: `{counts['observations']}`",
        f"- Corners: `{counts['corners']}`",
        f"- By split: `{counts['by_split']}`",
        f"- By target: `{counts['by_target']}`",
        f"- By camera role: `{counts['by_camera_role']}`",
        "",
        "## 외부 adapter 계약",
        "",
        "1. `external_baseline_observations.csv`의 `image_points_json`과 "
        "`object_points_json`을 그대로 사용한다.",
        "2. 새 detector, 새 corner refinement, 모델별 outlier 제거를 실행하지 않는다.",
        "3. `split=train`만으로 calibration/fit을 만들고, `split=test`는 평가에만 쓴다.",
        "4. main-method camera poses, joint optimizer 결과, Robot FK, Hand-Eye, fitted shared "
        "target pose를 입력으로 쓰지 않는다.",
        "5. 출력은 camera-pose 또는 prediction 파일과 adapter provenance SHA-256을 함께 남긴다.",
        "",
        "## 해석 한계",
        "",
        "이 package로 COLMAP/MATLAB/OpenCV 계열 baseline을 더 붙일 수 있지만, 결과가 "
        "좋거나 나쁘다고 해서 곧바로 external physical GT가 되는 것은 아니다. 역할은 "
        "custom optimizer dependency를 줄인 reference/sanity check다.",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_package(args: argparse.Namespace) -> None:
    manifest_path = Path(args.observation_manifest).resolve()
    table1_path = Path(args.table1_json).resolve()
    out_dir = Path(args.out_dir)
    session_root = Path(args.session_root).resolve()
    intrinsics_dir = Path(args.intrinsics_dir).resolve()
    manifest = load_json(manifest_path)
    table1 = load_json(table1_path)
    split = table1["protocol"]["split"]
    rows = selected_observations(manifest, args.policy, split)
    sources = validate_sources(manifest, rows, session_root, intrinsics_dir)
    counts = count_rows(rows)

    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "external_baseline_observations.csv"
    intrinsics_path = out_dir / "external_baseline_intrinsics.json"
    package_path = out_dir / "external_baseline_package.json"
    md_path = out_dir / "EXTERNAL_BASELINE_PACKAGE.md"

    write_csv(rows, csv_path)
    intrinsics_path.write_text(
        json.dumps(sources["intrinsics"], indent=2) + "\n",
        encoding="utf-8",
    )

    package = {
        "artifact_schema": "external_baseline_frozen_observation_package_v1",
        "feedback_id": 7,
        "role": "neutral_input_for_public_or_external_baseline_adapters",
        "status": "input_package_ready_adapter_execution_pending",
        "policy": args.policy,
        "source_manifest": {
            "path": str(manifest_path.relative_to(ROOT)),
            "sha256": sha256_file(manifest_path),
            "schema": manifest["schema"],
        },
        "source_table1": {
            "path": str(table1_path.relative_to(ROOT)),
            "sha256": sha256_file(table1_path),
        },
        "split": split,
        "counts": counts,
        "files": {
            "observations_csv": str(csv_path.relative_to(ROOT)),
            "intrinsics_json": str(intrinsics_path.relative_to(ROOT)),
            "readme": str(md_path.relative_to(ROOT)),
        },
        "source_validation": {
            "images_checked": len(sources["images"]),
            "intrinsics_checked": len(sources["intrinsics"]),
            "sha256_enforced": True,
        },
        "adapter_contract": {
            "allowed_inputs": [
                "frozen_RGB_image_paths_and_SHA256",
                "frozen_2d_image_points",
                "frozen_3d_object_points",
                "fixed_camera_intrinsics",
                "train_test_split",
            ],
            "forbidden_inputs": [
                "main_method_camera_poses",
                "joint_optimizer_state",
                "robot_FK",
                "hand_eye_transform",
                "fitted_shared_target_pose",
                "heldout_refit_or_model_dependent_gating",
            ],
            "adapter_targets": [
                "OpenCV_PnP_relative_pose",
                "MATLAB_or_other_public_multiview_baseline",
                "COLMAP_style_featureless_marker_adapter_if_supported",
            ],
            "not_external_gt": True,
            "not_sota_claim_by_itself": True,
        },
        "observation_digest": canonical_sha256(rows),
    }
    package_path.write_text(
        json.dumps(package, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    write_markdown(package, md_path)
    print(f"[DONE] wrote {package_path}")
    print(f"[DONE] wrote {csv_path}")
    print(f"[DONE] wrote {md_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--observation-manifest", default=str(DEFAULT_MANIFEST))
    parser.add_argument("--table1-json", default=str(DEFAULT_TABLE1))
    parser.add_argument("--session-root", default="data/session04/calib_train")
    parser.add_argument("--intrinsics-dir", default="intrinsics")
    parser.add_argument("--policy", default="standard", choices=("standard", "strict"))
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR))
    return parser.parse_args()


def main() -> None:
    build_package(parse_args())


if __name__ == "__main__":
    main()
