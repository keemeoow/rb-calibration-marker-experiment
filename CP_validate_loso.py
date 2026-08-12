#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CP_validate_loso.py — C1 held-out 지표의 leave-one-set-out(LOSO) 반복 검증.

단일 split(train 9 / test 4)의 held-out 숫자는 우연에 흔들릴 수 있다. 여기서는 **set 을
하나씩 빼서 test 로 쓰는 LOSO** 로 모든 fold 를 돌려, C1 의 세 솔버
(independent / unified_joint / joint_fk_fixed)와 세 예측(raw / +SE(3) / +Ridge)의
held-out RMSE 를 **평균 ± 표준편차**로 보고한다. 관측은 한 번만 검출하고 fold 마다
카메라만 다시 fit 하므로 CP_C1 을 13번 돌리는 것보다 훨씬 빠르다.

- 매 fold: correct_fk_cube_rotation(회전 prior 보정)을 **train set 으로만** 재추정 →
  train 으로 세 모델 solve → 빠진 set(=test)에서 큐브 위치를 FK 대비 RMSE 로 평가.
- 보정 4단(자유도 0→3→6→9)을 나란히 낸다: raw / +offset([1] 만·모델1 baseline) /
  +SE(3)(강체 6-DOF) / +Ridge([1,x,y] affine 9-DOF). offset 이 raw 와 같으면 감소분이
  단순 평균 offset 이 아니라 위치 의존 성분임을 뜻한다. 계수는 모두 train 으로만 학습.
- Ridge 계수 W 와 offset 벡터의 fold 간 안정성(평균 ± std), 축별 RMSE(rx/ry/rz)도 기록.
- FK 를 전혀 안 쓰는 지표(카메라 간 큐브중점 일치도)도 함께 낸다.

<<명령어>>
  PYTHONPATH= python CP_validate_loso.py --root_folder data/session --intrinsics_dir intrinsics
  # 결과 -> CP_result/validate_loso/{loso_summary.json, fig_CP_C1_loso.png}
  #   --min_test_events N  : test set 의 유효 이벤트가 N개 미만이면 그 fold 는 건너뜀(기본 1)
  #   --seeds "0,1,2"      : LOSO 대신 여러 seed 의 random holdout 로 반복(선택)
"""
import os
import json
import argparse
from typing import Dict, List, Optional

import numpy as np

import CP_common as cp
import Step3_calibration as s3
import CP_C1_unified_vs_independent as c1


def build_full_scene(root: str, intr: str, args):
    """CP_C1.main 의 로딩부와 동일하게 전체 Scene 을 한 번 만든다."""
    with open(os.path.join(root, "meta.json")) as f:
        meta = json.load(f)
    cfg, _ = cp.resolve_cube_config_for_run(
        root_folder=root, calib_dir="CP_result/validate_loso",
        cube_config_json=None, default_cfg=cp.get_default_cube_config())
    meta_cfg, _ = cp.load_cube_config_from_meta(root, default_cfg=cfg)
    reuse = cp.cube_configs_equivalent(meta_cfg, cfg)
    cube = cp.AprilTagCubeTarget(cfg)
    all_cams = sorted({int(k) for c in meta.get("captures", [])
                       for k, v in c.get("cams", {}).items() if v.get("saved")})
    gidx = args.gripper_cam_idx if args.gripper_cam_idx is not None else meta.get("gripper_cam_idx")
    if gidx is None:
        dm = os.path.join(intr, "device_map.json")
        if os.path.exists(dm):
            gidx = json.load(open(dm)).get("gripper_cam_idx")
    fixed = [c for c in all_cams if c != int(gidx)]
    K_map, D_map = {}, {}
    for ci in all_cams:
        K_map[ci], D_map[ci], _ = cp.load_intrinsics_with_depth_scale(intr, ci)
    event_to_set = {int(c["event_id"]): (int(cp.get_capture_set_index(c))
                    if cp.get_capture_set_index(c) is not None else None)
                    for c in meta.get("captures", []) if int(c.get("event_id", -1)) >= 0}
    robot_T = s3.load_robot_poses_from_meta(meta)
    set_priors = cp.load_nominal_set_cube_transforms(meta)
    set_pose6 = cp.load_nominal_set_cube_pose6(meta)
    pose_obs = cp.load_pose_observations(
        root=root, meta=meta, cube=cube, K_map=K_map, D_map=D_map, all_cam_ids=all_cams,
        gripper_cam_idx=int(gidx), reuse_stored_cube_candidates=reuse,
        max_err_fixed=float(args.max_err_fixed), max_err_gripper=float(args.max_err_gripper),
        min_aspect_fixed=0.0, min_aspect_gripper=0.35, gripper_min_markers=1,
        exclude_gripped=bool(args.exclude_gripped), fixed_min_markers=int(args.fixed_min_markers))
    sc = c1.build_scene(pose_obs, robot_T, set_priors, fixed, int(gidx), event_to_set)
    ref_cam = args.ref_fixed_cam_idx if args.ref_fixed_cam_idx is not None else fixed[0]
    return dict(sc=sc, pose_obs=pose_obs, set_priors=set_priors, set_pose6=set_pose6,
                event_to_set=event_to_set, fixed=fixed, gidx=int(gidx), ref_cam=int(ref_cam),
                meta=meta)


def run_fold(ctx, train_sets: List[int], test_sets: List[int], args) -> Optional[dict]:
    """한 fold: train 으로 fit, test 에서 세 모델 × (raw/+se3/+fk) 평가."""
    sc = ctx["sc"]
    # FK prior 회전 보정 (train 만)
    fixed_cube, _ = c1.correct_fk_cube_rotation(
        sc, ctx["pose_obs"], ctx["set_priors"], ctx["set_pose6"], ctx["event_to_set"],
        train_sets, ctx["ref_cam"], max_rot_error_deg=45.0)
    sc_f = c1.Scene(sc.fixed_cam_ids, sc.gripper_cam_idx, sc.obs_fixed, sc.obs_grip,
                    sc.bTg, fixed_cube, sc.sets)
    sc_fit = c1.subset_scene(sc_f, train_sets)
    if not sc_fit.sets or not sc_fit.obs_fixed:
        return None
    indep = c1.solve_independent(sc_fit, max_nfev=args.max_nfev, robust=True)
    joint = c1.solve_unified_joint(sc_fit, indep, anchor_weight=5.0, max_nfev=args.max_nfev)
    joint_fk = c1.solve_joint_fk_fixed(sc_fit, indep, max_nfev=args.max_nfev)
    out = {"train_sets": train_sets, "test_sets": test_sets, "methods": {}}
    for name, m in [("independent", indep), ("unified_joint", joint), ("joint_fk_fixed", joint_fk)]:
        W = c1.learn_fk_ridge(m, sc_fit, train_sets, lam=float(args.ridge_lambda))
        T_rig = c1.learn_fk_rigid(m, sc_fit, train_sets)
        off = c1.learn_fk_offset(m, sc_fit, train_sets)       # 모델1 baseline ([1] 만)
        # 축별(rx,ry,rz) pool 용 signed 오차와 스칼라 RMSE 를 한 번에 낸다.
        def _stage(**kw):
            e = c1.downstream_axis_errs(m, sc_f, test_sets, **kw)
            rmse = float(np.sqrt(np.mean(np.sum(e ** 2, axis=1)))) if e.shape[0] else None
            return rmse, e.tolist()
        raw, e_raw = _stage()
        offv, e_off = _stage(offset=off) if off is not None else (None, [])
        se3, e_se3 = _stage(T_rigid=T_rig) if T_rig is not None else (None, [])
        fk, e_fk = _stage(W=W) if W is not None else (None, [])
        out["methods"][name] = {
            "down_mm": raw, "down_offset_mm": offv, "down_se3_mm": se3, "down_fk_mm": fk,
            "ridge_W": (W.tolist() if W is not None else None),
            "offset_mm": (off * 1000.0).tolist() if off is not None else None,
            "axis_errs": {"raw": e_raw, "offset": e_off, "se3": e_se3, "fk": e_fk},
        }
    return out


def run_c3_fold(ctx, train_sets: List[int], test_sets: List[int], args) -> Optional[dict]:
    """C3 LOSO fold (closed-form, 고정 카메라만·event 단위).

    no-fk-prior 와 +correction 만 낸다 — 둘 다 조인트 최적화 없이 closed-form 으로
    faithful 하게 계산된다:
      - no-fk : robust pairwise 카메라(= 실데이터에서 03 이 폴백하는 02 와 동일) +
                train FK 병진 Kabsch 로 base 등록 → held-out event 큐브를 삼각측량해
                FK prior 대비 RMSE.
      - +corr : no-fk 예측의 train 잔차를 [1,x,y] Ridge 로 배워 held-out 에 적용.

    fk-prior(FK 를 solve 에 강제)는 event pose 수백 개를 동시에 푸는 조인트 solve 가
    본질이라 fold 당 ~100s 로 LOSO 반복이 비현실적이다. 그 방식의 결과(단일 split
    24.02mm, no-fk 보다 나쁨)는 CP_C3 헤드라인에 이미 있으므로 LOSO 에선 제외한다.
    """
    pose_obs, fixed, ref = ctx["pose_obs"], ctx["fixed"], ctx["ref_cam"]
    e2s, set_priors, set_pose6 = ctx["event_to_set"], ctx["set_priors"], ctx["set_pose6"]
    tr, te = set(train_sets), set(test_sets)
    train_obs = [o for o in pose_obs if int(o.cam) in fixed and e2s.get(int(o.event)) in tr]
    test_obs = [o for o in pose_obs if int(o.cam) in fixed and e2s.get(int(o.event)) in te]
    if len({int(o.cam) for o in train_obs}) < 2 or not test_obs:
        return None
    try:  # robust pairwise 카메라 + FK 병진 Kabsch base 등록 (모두 closed-form)
        T_base_ref, T_base_C, _, _, _, _ = cp.initialize_base_translation_anchored(
            pose_obs=train_obs, fixed_cam_ids=fixed, ref_cam=ref,
            set_priors={s: set_priors[s] for s in tr if s in set_priors},
            set_pose6={s: set_pose6[s] for s in tr if s in (set_pose6 or {})},
            event_to_set=e2s, max_trans_error_mm=1e9, max_rot_error_deg=45.0,
            disable_if_inconsistent=True)
    except Exception:
        return None
    cam = T_base_C  # base 프레임 고정 카메라

    T_obj_te = cp.estimate_object_poses_from_cams(test_obs, cam, fixed)
    nofk, _ = cp.prior_metrics(T_obj_te, e2s, set_priors)

    # +correction: Ridge [1,x,y] on train residuals → apply to held-out
    corr = None
    T_obj_tr = cp.estimate_object_poses_from_cams(train_obs, cam, fixed)
    X, Y = [], []
    for eid, T in T_obj_tr.items():
        s = e2s.get(eid)
        if s in tr and s in set_priors:
            t = T[:3, 3]
            X.append(np.array([1.0, t[0], t[1]]))
            Y.append(set_priors[s][:3, 3] - t)
    if len(X) >= 3:
        X = np.asarray(X); Y = np.asarray(Y)
        reg = float(args.ridge_lambda) * np.eye(3); reg[0, 0] = 0.0
        W = np.linalg.solve(X.T @ X + reg, X.T @ Y)
        errs = []
        for eid, T in T_obj_te.items():
            s = e2s.get(eid)
            if s in te and s in set_priors:
                t = T[:3, 3] + np.array([1.0, T[0, 3], T[1, 3]]) @ W
                errs.append(float(np.linalg.norm(t - set_priors[s][:3, 3]) * 1000.0))
        if errs:
            corr = float(np.sqrt(np.mean(np.square(errs))))
    return {"train_sets": train_sets, "test_sets": test_sets,
            "no_fk_prior": nofk, "fk_prior": None, "fk_prior_correction": corr}


def agg(vals: List[Optional[float]]) -> dict:
    v = np.array([x for x in vals if x is not None], dtype=float)
    if not v.size:
        return {"mean": None, "std": None, "n": 0}
    return {"mean": float(v.mean()), "std": float(v.std(ddof=1) if v.size > 1 else 0.0),
            "median": float(np.median(v)), "min": float(v.min()), "max": float(v.max()),
            "n": int(v.size)}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root_folder", required=True)
    ap.add_argument("--intrinsics_dir", required=True)
    ap.add_argument("--out_dir", default=os.path.join("CP_result", "validate_loso"))
    ap.add_argument("--gripper_cam_idx", type=int, default=None)
    ap.add_argument("--ref_fixed_cam_idx", type=int, default=None)
    ap.add_argument("--max_err_fixed", type=float, default=3.0)
    ap.add_argument("--max_err_gripper", type=float, default=5.0)
    ap.add_argument("--fixed_min_markers", type=int, default=2)
    ap.add_argument("--exclude_gripped", type=lambda v: str(v).lower() not in ("0", "false", "no"),
                    default=True)
    ap.add_argument("--max_nfev", type=int, default=200)
    ap.add_argument("--ridge_lambda", type=float, default=1e-3)
    ap.add_argument("--min_test_events", type=int, default=1)
    ap.add_argument("--seeds", type=str, default="",
                    help="비우면 LOSO. 지정하면 각 seed 로 holdout_frac 0.3 random split 반복.")
    ap.add_argument("--holdout_frac", type=float, default=0.3)
    args = ap.parse_args()

    out_dir = cp.ensure_dir(args.out_dir)
    ctx = build_full_scene(args.root_folder, args.intrinsics_dir, args)
    sets = list(ctx["sc"].sets)
    print(f"[INFO] fixed={ctx['fixed']} gripper=cam{ctx['gidx']} ref=cam{ctx['ref_cam']}  sets={sets}")

    # fold 구성
    folds = []
    if str(args.seeds).strip():
        seeds = [int(s) for s in str(args.seeds).replace(";", ",").split(",") if s.strip()]
        for sd in seeds:
            test = c1.resolve_test_sets(sets, "", float(args.holdout_frac), sd)
            folds.append(("seed%d" % sd, [s for s in sets if s not in set(test)], test))
        mode = f"random holdout (frac={args.holdout_frac}, seeds={seeds})"
    else:
        for s in sets:
            folds.append(("loso_set%d" % s, [x for x in sets if x != s], [s]))
        mode = "leave-one-set-out"
    print(f"[INFO] validation mode: {mode}  ({len(folds)} folds)")

    fold_results = []
    stages = ("raw", "offset", "se3", "fk")
    per_method = {m: {"down_mm": [], "down_offset_mm": [], "down_se3_mm": [], "down_fk_mm": [],
                      "ridge_W": [], "offset_mm": [],
                      "axis": {s: [] for s in stages}}
                  for m in ("independent", "unified_joint", "joint_fk_fixed")}
    for tag, tr, te in folds:
        r = run_fold(ctx, tr, te, args)
        if r is None:
            print(f"  {tag}: skipped (no train obs)")
            continue
        # test 이벤트 수 체크
        r["tag"] = tag
        fold_results.append(r)
        line = f"  {tag} (test={te}):"
        for m in per_method:
            md = r["methods"][m]
            for k in ("down_mm", "down_offset_mm", "down_se3_mm", "down_fk_mm"):
                per_method[m][k].append(md[k])
            if md["ridge_W"] is not None:
                per_method[m]["ridge_W"].append(np.array(md["ridge_W"]))
            if md.get("offset_mm") is not None:
                per_method[m]["offset_mm"].append(np.array(md["offset_mm"]))
            for s in stages:                                   # 축별 pool 용 signed 오차 누적
                e = md.get("axis_errs", {}).get(s) or []
                if e:
                    per_method[m]["axis"][s].append(np.asarray(e, float))
            line += (f"  {m[:4]} raw={_f(md['down_mm'])}/off={_f(md['down_offset_mm'])}"
                     f"/se3={_f(md['down_se3_mm'])}/fk={_f(md['down_fk_mm'])}")
        print(line)

    # 집계
    summary = {"mode": mode, "n_folds": len(fold_results), "sets": sets, "methods": {}}
    print("\n" + "=" * 108)
    print(f"LOSO 요약  ({len(fold_results)} folds, mean ± std [min..max] mm)  —  보정 자유도 0→3→6→9")
    print("=" * 108)
    print(f"{'method':16s} {'raw (0)':>21s} {'+offset[1] (3)':>21s} {'+SE(3) (6)':>21s} {'+Ridge[1,x,y] (9)':>21s}")
    print("-" * 108)
    for m in per_method:
        a_raw = agg(per_method[m]["down_mm"])
        a_off = agg(per_method[m]["down_offset_mm"])
        a_se3 = agg(per_method[m]["down_se3_mm"])
        a_fk = agg(per_method[m]["down_fk_mm"])
        Ws = per_method[m]["ridge_W"]
        W_mean = (np.mean(Ws, axis=0).tolist() if Ws else None)
        W_std = (np.std(Ws, axis=0).tolist() if Ws else None)
        offs = per_method[m]["offset_mm"]
        off_mean = (np.mean(offs, axis=0).tolist() if offs else None)
        off_std = (np.std(offs, axis=0).tolist() if offs else None)
        # 축별 RMS (fold 간 pool)
        axis = {}
        for s in stages:
            arrs = per_method[m]["axis"][s]
            if arrs:
                a = np.vstack(arrs)
                rms = np.sqrt(np.mean(a ** 2, axis=0))
                axis[s] = {"rx": float(rms[0]), "ry": float(rms[1]), "rz": float(rms[2]),
                           "norm": float(np.sqrt(np.mean(np.sum(a ** 2, axis=1)))), "n": int(a.shape[0])}
            else:
                axis[s] = None
        summary["methods"][m] = {"raw": a_raw, "offset": a_off, "se3": a_se3, "fk": a_fk,
                                 "ridge_W_mean": W_mean, "ridge_W_std": W_std,
                                 "offset_mm_mean": off_mean, "offset_mm_std": off_std,
                                 "axis_rms": axis}
        print(f"{m:16s} {_ms(a_raw):>21s} {_ms(a_off):>21s} {_ms(a_se3):>21s} {_ms(a_fk):>21s}")

    # 축별 RMSE (rx/ry/rz) — fold 간 pool
    print("\n" + "=" * 108)
    print("축별 RMSE (mm)  —  rx / ry / rz / |r|  (fold 간 pool)")
    print("=" * 108)
    print(f"{'method':16s} {'stage':>8s} {'rx':>9s} {'ry':>9s} {'rz':>9s} {'|r|':>9s}")
    print("-" * 108)
    for m in per_method:
        for s in stages:
            ax = summary["methods"][m]["axis_rms"].get(s)
            if ax is None:
                continue
            print(f"{m:16s} {s:>8s} {ax['rx']:>9.2f} {ax['ry']:>9.2f} {ax['rz']:>9.2f} {ax['norm']:>9.2f}")
        print("-" * 108)

    # ── C3 LOSO: no-fk / fk-prior / +correction (event 단위, 고정 카메라만) ──
    c3_keys = ["no_fk_prior", "fk_prior", "fk_prior_correction"]
    c3_acc = {k: [] for k in c3_keys}
    c3_folds = []
    print("\n" + "=" * 92)
    print("C3 LOSO — held-out FK 위치 RMSE (no-fk / fk-prior / +correction)")
    print("=" * 92)
    for tag, tr, te in folds:
        r3 = run_c3_fold(ctx, tr, te, args)
        if r3 is None:
            continue
        r3["tag"] = tag
        c3_folds.append(r3)
        for k in c3_keys:
            c3_acc[k].append(r3[k])
        print(f"  {tag} (test={te}): no-fk={_f(r3['no_fk_prior'])}  "
              f"fk-prior={_f(r3['fk_prior'])}  +corr={_f(r3['fk_prior_correction'])}")
    c3_summary = {"mode": mode, "n_folds": len(c3_folds)}
    for k in c3_keys:
        c3_summary[k] = agg(c3_acc[k])
    print("-" * 92)
    print(f"{'':16s} {'no-fk-prior':>22s} {'fk-prior':>22s} {'+correction':>22s}")
    print(f"{'mean±std[min..max]':16s} {_ms(c3_summary['no_fk_prior']):>22s} "
          f"{_ms(c3_summary['fk_prior']):>22s} {_ms(c3_summary['fk_prior_correction']):>22s}")

    with open(os.path.join(out_dir, "loso_summary.json"), "w") as f:
        json.dump({"c1": {"summary": summary, "folds": fold_results},
                   "c3": {"summary": c3_summary, "folds": c3_folds}},
                  f, indent=2, ensure_ascii=False)
    print(f"\n[DONE] {os.path.join(out_dir, 'loso_summary.json')}")
    try:
        _make_figure(summary, os.path.join(out_dir, "fig_CP_C1_loso.png"))
        print(f"[DONE] {os.path.join(out_dir, 'fig_CP_C1_loso.png')}")
        _make_c3_figure(c3_summary, os.path.join(out_dir, "fig_CP_C3_loso.png"))
        print(f"[DONE] {os.path.join(out_dir, 'fig_CP_C3_loso.png')}")
    except Exception as e:
        print(f"[WARN] figure skipped: {e}")


def _f(x):
    return "NA" if x is None else f"{x:.2f}"


def _ms(a):
    if a["mean"] is None:
        return "NA"
    return f"{a['mean']:.2f}±{a['std']:.2f}[{a['min']:.1f}..{a['max']:.1f}]"


def _make_figure(summary: dict, path: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": ["Apple SD Gothic Neo", "NanumGothic", "DejaVu Sans"],
                         "axes.unicode_minus": False, "figure.facecolor": "#fcfcfb",
                         "axes.facecolor": "#fcfcfb"})
    methods = ["independent", "unified_joint", "joint_fk_fixed"]
    series = [("raw", "원출력 (raw)", "#cdd8e6"), ("offset", "+ offset [1]", "#9fb8d4"),
              ("se3", "+ SE(3)", "#2a78d6"), ("fk", "+ Ridge [1,x,y]", "#104281")]
    fig, ax = plt.subplots(figsize=(10.6, 5.6))
    n = len(series)
    gw, gap = 0.78, 0.03
    bw = (gw - gap * (n - 1)) / n
    for j, (key, label, color) in enumerate(series):
        xs, means, stds = [], [], []
        for i, m in enumerate(methods):
            a = summary["methods"][m][key]
            xs.append(i - gw / 2 + bw / 2 + j * (bw + gap))
            means.append(a["mean"] or 0.0)
            stds.append(a["std"] or 0.0)
        ax.bar(xs, means, width=bw, color=color, yerr=stds, capsize=4,
               error_kw=dict(ecolor="#333", lw=1.1), zorder=3, label=label)
        for x, mnv, sd in zip(xs, means, stds):
            ax.annotate(f"{mnv:.1f}±{sd:.1f}", (x, mnv + sd), xytext=(0, 3),
                        textcoords="offset points", ha="center", va="bottom", fontsize=8.5)
    ax.axhline(5.0, color="#8a8a86", lw=1.1, ls=(0, (4, 3)), zorder=2)
    ax.set_xticks(range(len(methods)))
    ax.set_xticklabels(methods, fontsize=10.5)
    ax.set_ylabel("held-out 큐브 위치 RMSE (mm)", fontsize=10)
    ax.set_title(f"C1 — LOSO 반복 검증 ({summary['n_folds']} folds, mean ± std)",
                 fontsize=13.5, loc="left", pad=12)
    ax.legend(frameon=False, fontsize=9, loc="upper right")
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    ax.grid(axis="y", color="#e6e5e1", lw=0.8); ax.set_axisbelow(True)
    fig.text(0.008, -0.01,
             f"data/session · {summary['mode']} · 오차막대 = fold 간 표준편차. "
             "보정(+offset/+SE(3)/+Ridge)은 매 fold train 에서만 학습해 held-out set 에 적용. "
             "+offset≈raw ⇒ 감소분은 평균 offset 이 아닌 위치 의존 성분.",
             fontsize=8.5, color="#52514e", va="top")
    fig.savefig(path, dpi=160, bbox_inches="tight", facecolor=fig.get_facecolor())


def _make_c3_figure(c3: dict, path: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": ["Apple SD Gothic Neo", "NanumGothic", "DejaVu Sans"],
                         "axes.unicode_minus": False, "figure.facecolor": "#fcfcfb",
                         "axes.facecolor": "#fcfcfb"})
    keys = [("no_fk_prior", "no-fk-prior\n(vision only)", "#6b7a86"),
            ("fk_prior_correction", "no-fk\n+ Ridge 후보정", "#1baf7a")]
    keys = [(k, l, c) for (k, l, c) in keys if c3.get(k, {}).get("mean") is not None]
    fig, ax = plt.subplots(figsize=(7.0, 5.6))
    for i, (k, label, color) in enumerate(keys):
        a = c3[k]
        ax.bar(i, a["mean"] or 0.0, width=0.55, color=color, yerr=(a["std"] or 0.0),
               capsize=4, error_kw=dict(ecolor="#333", lw=1.1), zorder=3)
        ax.annotate(f"{a['mean']:.1f}±{a['std']:.1f}", (i, a["mean"] + (a["std"] or 0)),
                    xytext=(0, 3), textcoords="offset points", ha="center",
                    va="bottom", fontsize=9.5, fontweight="bold")
    ax.set_xticks(range(len(keys)))
    ax.set_xticklabels([k[1] for k in keys], fontsize=10)
    ax.set_xlim(-0.6, len(keys) - 0.4)
    ax.set_ylabel("held-out FK 위치 RMSE (mm)", fontsize=10)
    ax.set_title(f"C3 — LOSO 반복 검증 ({c3['n_folds']} folds, mean ± std)",
                 fontsize=13.5, loc="left", pad=12)
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    ax.grid(axis="y", color="#e6e5e1", lw=0.8); ax.set_axisbelow(True)
    fig.text(0.008, -0.02,
             f"data/session · {c3['mode']} · event 단위·고정 카메라만·closed-form · 오차막대 = fold 간 표준편차.\n"
             "FK 프록시 대비 값(절대 정확도 아님). fk-prior(solve 강제, 단일 split 24.02mm)는 조인트 solve 가 "
             "fold 당 ~100s 라 LOSO 제외.", fontsize=8.5, color="#52514e", va="top")
    fig.savefig(path, dpi=160, bbox_inches="tight", facecolor=fig.get_facecolor())


if __name__ == "__main__":
    main()
