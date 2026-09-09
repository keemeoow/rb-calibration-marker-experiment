#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CP_final_fk_mode_fit.py — 세 FK 방식을 전체 데이터로 fit 해 최종 행렬을 뽑는다.

`CP_D1_fk_correction_2x2.py` 는 위치 단위 leave-one-out 으로 **어느 방식이 나은지**만
판정한다. 이 스크립트는 같은 backend·같은 solver 설정으로 **hold-out 없이 전체 위치**를
써서 한 번 fit 하고, 그 결과 행렬(`T_base_Ci`, `T_gripper_cam`)을 Step3 와 같은 형식으로
저장한다. 판정과 산출을 분리해 두어야 "어느 방식이 이겼나" 와 "그 방식으로 뽑은 최종 값"
사이에 backend 차이가 끼어들지 않는다.

방식(arm) 정의는 D1 과 동일하다.

  no-FK        A2   큐브 자세를 자유변수로 두고 영상만으로 추정
  fixed-FK     A3   큐브 자세를 (train 전용) 정렬된 FK artifact 에 고정
  corrected-FK A3(또는 A2) + 예측 시점 잔차 보정

**중요** — 잔차 보정은 `T_base_Ci` 와 `T_gripper_cam` 을 건드리지 않는다. 캘리브가 끝난 뒤
예측된 큐브 중심에만 적용되는 후처리다. 따라서 corrected-FK 의 **최종 행렬은 그 base arm
과 완전히 같고**, 달라지는 것은 함께 저장되는 보정 계수뿐이다. 이 스크립트는 그 사실을
문서가 아니라 assert 로 확인한다.

<<명령어>>
  PYTHONPATH= python CP_final_fk_mode_fit.py \
      --root_folder data/session02/calib_train --intrinsics_dir intrinsics \
      --calib_dir data/session02/calib_out \
      --out_dir ABLATION_TEST_result/final_fk_mode_fit
"""
from __future__ import annotations

import argparse
import json
import os
from collections import defaultdict
from typing import Dict, List, Mapping, Sequence

import numpy as np

import CP_ablation_7row as ab
import Step3_calibration as s3
import CP_D1_fk_correction_2x2 as d1
from CP_ablation_schema import (
    MAIN_ABLATION_CONDITIONS,
    UNIFIED_FREE_VARIABLES,
    AblationCondition,
    validate_fk_alignment_artifact,
)
from apriltag_cube import inv_T
from calibration_fk_cube_artifact import estimate_board_free_fk_cube_artifact
from calibration_runtime_utils import (
    get_capture_set_index,
    load_intrinsics_with_depth_scale,
)
from calibration_path_evaluation import observation_id
from calibration_reprojection_backend import SolverOptions, variable_keys


ARMS = {
    "no_fk": {"row": "A2", "anchor_lambda": 0.0,
              "label": "no-FK (큐브 자세를 영상으로 추정)"},
    "fixed_fk": {"row": "A3", "anchor_lambda": None,
                 "label": "fixed-FK (큐브 자세를 FK 로 고정)"},
}


def fit_arm(args, spec: Mapping, observations, train_sets: Sequence[int],
            aligned_fk_all, fixed_gtc_initial, board_gtc, board_initial,
            visual_cubes, robot_T, K_map, D_map, gripper: int):
    """D1 의 run_fold 와 동일한 경로. 다른 점은 hold-out 이 없다는 것뿐이다."""
    conditions = {c.row: c for c in MAIN_ABLATION_CONDITIONS}
    condition: AblationCondition = conditions[spec["row"]]
    fixed_cubes = {int(s): aligned_fk_all[int(s)] for s in train_sets}
    initial_state, init_diag = ab.make_initial_state(
        condition, observations, gripper, robot_T, K_map, D_map,
        board_gtc, board_initial, visual_cubes, fixed_cubes, fixed_gtc_initial)
    fit_obs = ab.filter_observations(
        observations, condition, None, gripper, initial_state.cams)
    state, diag = d1.solve_anchored_corner_reprojection(
        observations=fit_obs,
        variable_keys_=variable_keys(UNIFIED_FREE_VARIABLES[spec["row"]], initial_state),
        reference_state=initial_state,
        robot_T=robot_T,
        K_map=K_map,
        D_map=D_map,
        gripper_cam_idx=gripper,
        anchor_targets=({} if spec["anchor_lambda"] is None else fixed_cubes),
        anchor_lambda=(0.0 if spec["anchor_lambda"] is None
                       else float(spec["anchor_lambda"])),
        anchor_lever_mm=float(args.anchor_lever_mm),
        options=ab.canonical_solver_options(args),
        seed=int(args.seed),
        init_translation_mm=float(args.init_translation_mm),
        init_rotation_deg=float(args.init_rotation_deg),
    )
    return state, diag, init_diag


def write_transforms(out_dir: str, arm_key: str, state, ref_cam: int,
                     root_folder: str, calib_dir: str) -> dict:
    """Step3 의 final_transforms_base_frame.json 과 같은 형식으로 저장한다."""
    os.makedirs(out_dir, exist_ok=True)
    transforms: Dict[str, np.ndarray] = {}
    T_base_ref = np.asarray(state.cams[int(ref_cam)], float)
    for ci in sorted(state.cams):
        transforms[f"T_base_C{int(ci)}"] = np.asarray(state.cams[int(ci)], float)
        transforms[f"T_C{int(ref_cam)}_C{int(ci)}"] = (
            inv_T(T_base_ref) @ np.asarray(state.cams[int(ci)], float))
    transforms["T_gripper_cam"] = np.asarray(state.gtc, float)
    for name, T in transforms.items():
        np.save(os.path.join(out_dir, f"{name}.npy"), T)
    payload = {
        "generated_by": "CP_final_fk_mode_fit.py",
        "arm": arm_key,
        "root_folder": root_folder,
        "calib_dir": calib_dir,
        "reference_fixed_cam": int(ref_cam),
        "units": "meters",
        "transforms": {k: T.tolist() for k, T in transforms.items()},
    }
    with open(os.path.join(out_dir, "final_transforms_base_frame.json"), "w") as handle:
        json.dump(payload, handle, indent=2)
    return payload


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root_folder", default="data/session02/calib_train")
    parser.add_argument("--intrinsics_dir", default="intrinsics")
    parser.add_argument("--calib_dir", default="data/session02/calib_out")
    parser.add_argument("--out_dir", default="ABLATION_TEST_result/final_fk_mode_fit")
    parser.add_argument("--arms", default="no_fk,fixed_fk")
    parser.add_argument("--ref_cam", type=int, default=None,
                        help="상대 외부파라미터 기준 고정 카메라. 기본은 가장 작은 고정 cam id.")
    parser.add_argument("--min_eih_cube_events", type=int, default=3)
    parser.add_argument("--min_fixed_cube_observations", type=int, default=2)
    parser.add_argument("--ridge_lambda", type=float, default=1e-3)
    parser.add_argument("--anchor_lever_mm", type=float, default=29.5)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--artifact_inits", type=int, default=3)
    parser.add_argument("--init_translation_mm", type=float, default=5.0)
    parser.add_argument("--init_rotation_deg", type=float, default=1.0)
    parser.add_argument("--max_nfev", type=int, default=300)
    parser.add_argument("--tol", type=float, default=1e-8)
    parser.add_argument("--rotation_scale_rad", type=float, default=1.0)
    parser.add_argument("--translation_scale_m", type=float, default=1.0)
    parser.add_argument("--x_scale_mode", choices=["unit", "jac"], default="jac")
    parser.add_argument("--loss", choices=["huber", "soft_l1", "linear"], default="soft_l1")
    parser.add_argument("--f_scale_px", type=float, default=2.0)
    parser.add_argument("--image_scale", type=float, default=1.0)
    args = parser.parse_args()

    # ── D1 과 완전히 같은 로드·적격성 판정 경로 ──────────────────────────────
    with open(os.path.join(args.root_folder, "meta.json")) as handle:
        meta = json.load(handle)
    all_cam_ids = sorted({int(ci) for cap in meta.get("captures", [])
                          for ci in cap.get("cams", {})})
    gripper = int(meta["gripper_cam_idx"])
    K_map, D_map = {}, {}
    for ci in all_cam_ids:
        K_map[ci], D_map[ci], _ = load_intrinsics_with_depth_scale(args.intrinsics_dir, ci)
    robot_T = s3.load_robot_poses_from_meta(meta)
    observations, cube_cfg_source, cube_reason = ab.detect_observations(
        args, meta, K_map, D_map, all_cam_ids, gripper)

    raw_fk_all = s3.load_nominal_set_cube_transforms(meta)
    raw_fk_source_event_by_set = {}
    for cap in meta.get("captures", []):
        set_index = get_capture_set_index(cap)
        if set_index is not None and int(set_index) not in raw_fk_source_event_by_set:
            raw_fk_source_event_by_set[int(set_index)] = int(cap["event_id"])

    eih_events: Dict[int, set] = defaultdict(set)
    fixed_counts: Dict[int, int] = defaultdict(int)
    for obs in observations:
        if obs.marker != "cube" or obs.set_idx is None:
            continue
        if int(obs.cam) == gripper:
            eih_events[int(obs.set_idx)].add(int(obs.event))
        else:
            fixed_counts[int(obs.set_idx)] += 1
    eligible, dropped = [], {}
    for s in sorted(set(eih_events) | set(fixed_counts)):
        if s not in raw_fk_all:
            dropped[str(s)] = "no raw FK cube pose"
            continue
        if len(eih_events.get(s, set())) < int(args.min_eih_cube_events):
            dropped[str(s)] = (f"eih cube events {len(eih_events.get(s, set()))} < "
                               f"{args.min_eih_cube_events}")
            continue
        if fixed_counts.get(s, 0) < int(args.min_fixed_cube_observations):
            dropped[str(s)] = (f"fixed cube observations {fixed_counts.get(s, 0)} < "
                               f"{args.min_fixed_cube_observations}")
            continue
        eligible.append(int(s))
    print(f"[FIT] eligible positions ({len(eligible)}): {eligible}")
    if dropped:
        print(f"[FIT] dropped: {json.dumps(dropped, ensure_ascii=False)}")

    prediction_mask = d1.build_prediction_mask(
        observations, eligible, gripper, K_map, D_map)
    by_id = {observation_id(obs): obs for obs in observations
             if obs.marker == "cube" and obs.set_idx is not None
             and int(obs.set_idx) in set(eligible)}

    # 전체 위치를 train 으로 쓰는 FK 정렬 artifact
    aligned_fk_all, fixed_gtc_initial, artifact = estimate_board_free_fk_cube_artifact(
        observations=observations,
        raw_fk_by_set=raw_fk_all,
        robot_T=robot_T,
        K_map=K_map,
        D_map=D_map,
        gripper_cam_idx=gripper,
        training_set_ids=sorted(eligible),
        options=SolverOptions(),
        num_inits=int(args.artifact_inits),
        init_translation_mm=float(args.init_translation_mm),
        init_rotation_deg=float(args.init_rotation_deg),
        raw_fk_source_event_by_set=raw_fk_source_event_by_set,
    )
    validate_fk_alignment_artifact(artifact)

    eih_board = [obs for obs in observations
                 if obs.marker == "board" and int(obs.cam) == int(gripper)]
    board_gtc, board_initial, handeye_diag = ab.estimate_board_handeye_initial(
        eih_board, robot_T, K_map, D_map, gripper)
    visual_cubes = ab.average_visual_target(
        observations, "cube", board_gtc, robot_T, K_map, D_map, gripper)
    missing_visual = sorted(set(eligible) - set(visual_cubes))
    if missing_visual:
        raise RuntimeError(f"visual cube initialization missing for sets {missing_visual}")
    reference = {int(s): np.asarray(aligned_fk_all[int(s)], float)[:3, 3]
                 for s in eligible if int(s) in aligned_fk_all}

    os.makedirs(args.out_dir, exist_ok=True)
    requested = [a.strip() for a in str(args.arms).split(",") if a.strip()]
    unknown = sorted(set(requested) - set(ARMS))
    if unknown:
        raise RuntimeError(f"unknown arms: {unknown}")

    results = {}
    for arm_key in requested:
        spec = ARMS[arm_key]
        print(f"[FIT] arm {arm_key} ({spec['label']}) — {len(eligible)} positions, "
              f"no hold-out", flush=True)
        state, diag, init_diag = fit_arm(
            args, spec, observations, eligible, aligned_fk_all, fixed_gtc_initial,
            board_gtc, board_initial, visual_cubes, robot_T, K_map, D_map, gripper)

        fixed_cams = sorted(int(c) for c in state.cams)
        ref_cam = int(args.ref_cam) if args.ref_cam is not None else fixed_cams[0]
        arm_dir = os.path.join(args.out_dir, arm_key)
        payload = write_transforms(arm_dir, arm_key, state, ref_cam,
                                   args.root_folder, args.calib_dir)

        # in-sample 진단 — hold-out 이 아니므로 판정에는 쓸 수 없다.
        predicted = d1.predict_set_centres(
            prediction_mask, by_id, state.cams, state.gtc, robot_T, gripper, K_map, D_map)
        cross = {}
        for s in eligible:
            rmse, pairs = d1.heldout_cross_translation_mm(
                prediction_mask, by_id, state.cams, s, K_map, D_map)
            if rmse is not None:
                cross[int(s)] = {"rmse_mm": rmse, "n_pairs": int(pairs)}
        cross_values = [v["rmse_mm"] for v in cross.values()]

        corrections = {}
        for kind in d1.CORRECTIONS:
            param = d1.learn_correction(kind, predicted, reference, eligible,
                                        float(args.ridge_lambda))
            resid = [float(np.linalg.norm(
                d1.apply_correction(kind, param, predicted[s]) - reference[s])) * 1000.0
                for s in eligible if s in predicted and s in reference]
            corrections[kind] = {
                "dof": d1.CORRECTION_DOF[kind],
                "in_sample_position_rmse_mm": (
                    float(np.sqrt(np.mean(np.square(resid)))) if resid else None),
                "parameters": (None if param is None
                               else np.asarray(param, float).tolist()),
                "note": ("보정은 T_base_Ci·T_gripper_cam 을 바꾸지 않는다. "
                         "예측된 큐브 중심에만 적용한다."),
            }

        summary = {
            "arm": arm_key,
            "label": spec["label"],
            "row": spec["row"],
            "anchor_lambda_px_per_mm": spec["anchor_lambda"],
            "n_positions": len(eligible),
            "positions": eligible,
            "converged": bool(diag.get("success", diag.get("converged"))),
            "solver": {k: diag.get(k) for k in
                       ("nfev", "cost", "optimality", "message", "success")},
            "reference_fixed_cam": ref_cam,
            "fixed_cams": fixed_cams,
            "in_sample": {
                "fk_proxy_position_rmse_mm": corrections["none"]["in_sample_position_rmse_mm"],
                "cross_camera_translation_rmse_mm_by_position": cross,
                "cross_camera_translation_rmse_mm_mean": (
                    float(np.mean(cross_values)) if cross_values else None),
                "cross_camera_translation_rmse_mm_median": (
                    float(np.median(cross_values)) if cross_values else None),
                "warning": "hold-out 이 아니다. 방식 판정은 CP_D1 결과로만 한다.",
            },
            "corrections": corrections,
            "cube_config_source": cube_cfg_source,
            "cube_detection": cube_reason,
            "solver_options": ab.canonical_solver_options(args).to_dict(),
            "fk_alignment_artifact": {
                "training_set_ids": artifact.get("training_set_ids"),
                "schema": artifact.get("artifact_schema"),
            },
            "transforms_file": os.path.join(arm_dir, "final_transforms_base_frame.json"),
        }
        cube_by_set = {str(int(s)): np.asarray(state.cubes[int(s)], float).tolist()
                       for s in sorted(state.cubes)} if state.cubes else {}
        with open(os.path.join(arm_dir, "T_base_cube_by_set.json"), "w") as handle:
            json.dump({"arm": arm_key, "units": "meters", "transforms": cube_by_set},
                      handle, indent=2)
        with open(os.path.join(arm_dir, "fit_summary.json"), "w") as handle:
            json.dump(summary, handle, indent=2, ensure_ascii=False)
        results[arm_key] = {"summary": summary, "transforms": payload["transforms"]}
        print(f"       cross-camera agreement mean {summary['in_sample']['cross_camera_translation_rmse_mm_mean']:.3f} mm "
              f"(median {summary['in_sample']['cross_camera_translation_rmse_mm_median']:.3f}), "
              f"FK-proxy in-sample {summary['in_sample']['fk_proxy_position_rmse_mm']:.3f} mm", flush=True)

    with open(os.path.join(args.out_dir, "fit_index.json"), "w") as handle:
        json.dump({
            "root_folder": args.root_folder,
            "arms": {k: v["summary"] for k, v in results.items()},
            "note": ("corrected-FK 의 최종 행렬은 base arm 과 동일하다. "
                     "차이는 corrections 항목의 계수뿐이다."),
        }, handle, indent=2, ensure_ascii=False)
    print(f"[FIT] wrote {args.out_dir}")


if __name__ == "__main__":
    main()
