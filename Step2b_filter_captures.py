#!/usr/bin/env python3
"""Step 2b: 저장된 캡처를 사후에 품질 기준으로 거른다.

Step2 의 게이트를 풀고 전부 저장한 뒤, 여기서 기준을 정해 쓸 프레임을 고른다.
촬영 중에는 기준을 확정하기 어렵고(한 번 버린 프레임은 되돌릴 수 없다), 반대로
기준 없이 전부 쓰면 발산한 pose 가 캘리브레이션에 섞인다. 저장은 관대하게, 선별은
사후에 — 그래야 기준을 바꿔가며 몇 장이 남는지 보고 정할 수 있다.

판정에 필요한 값은 촬영 시점에 이미 `meta.json` 에 기록되어 있다(게이트 통과 여부와
무관하게 남는다). 이미지 재검출 없이 그 기록만으로 재평가하므로 빠르고, 원본은
건드리지 않는다.

    # 현재 데이터가 기준별로 몇 장 남는지만 본다 (파일 안 씀)
    python Step2b_filter_captures.py --root data/session02/calib_train

    # 기준을 정해 거른 meta 를 쓴다
    python Step2b_filter_captures.py --root data/session02/calib_train \
        --max_span_ms 120 --min_fixed_pnp_ok 2 --out meta.filtered.json

원본 `meta.json` 은 절대 덮어쓰지 않는다. 출력은 별도 파일이고, 각 캡처에 `qc`
블록(통과 여부와 탈락 사유)이 붙는다.
"""
from __future__ import annotations

import argparse
import collections
import json
import os
import sys
from typing import Dict, List, Optional, Tuple

# 촬영 시점 기본 게이트와 같은 값. 사후 선별의 출발점을 촬영 계약과 일치시킨다.
DEFAULT_MAX_SPAN_MS = 120.0
DEFAULT_MAX_PNP_REPROJ_PX = 2.0
DEFAULT_MIN_FIXED_PNP_OK = 1
DEFAULT_MAX_ROI_CLIP = 0.05


def _f(value, default=None) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def evaluate(cap: dict, gripper_idx: Optional[int], args) -> Tuple[bool, List[str]]:
    """캡처 하나를 판정한다. 반환: (통과 여부, 탈락 사유 목록).

    고정 카메라만 본다. 그리퍼캠은 A 에서 큐브를 자주 놓치고 B 에서는 큐브에 가려
    아예 못 보므로, 그 관측을 필수로 걸면 멀쩡한 프레임까지 버린다.
    """
    gate = cap.get("capture_gate") or {}
    per_cam = gate.get("per_camera") or {}
    reasons: List[str] = []

    span = _f(gate.get("capture_span_ms"))
    if args.max_span_ms > 0:
        if span is None:
            reasons.append("span 미기록")
        elif span > args.max_span_ms:
            reasons.append("span {:.0f}ms > {:.0f}".format(span, args.max_span_ms))

    fixed_ok = 0
    for ci, pc in per_cam.items():
        if gripper_idx is not None and str(ci) == str(gripper_idx):
            continue
        if not pc.get("cube_pnp_ok"):
            continue
        reproj = _f(pc.get("cube_pnp_reproj_mean_px"))
        if args.max_pnp_reproj_px > 0 and reproj is not None \
                and reproj > args.max_pnp_reproj_px:
            continue
        fixed_ok += 1
    if fixed_ok < args.min_fixed_pnp_ok:
        reasons.append("고정캠 PnP {} < {}".format(fixed_ok, args.min_fixed_pnp_ok))

    if args.max_roi_clip > 0:
        clipped = []
        for ci, pc in per_cam.items():
            v = _f(pc.get("roi_clip_frac_median"))
            if v is not None and v > args.max_roi_clip:
                clipped.append(int(ci))
        if clipped:
            reasons.append("ROI 포화 cam{}".format(clipped))

    if args.min_roi_sharpness > 0:
        blurred = []
        for ci, pc in per_cam.items():
            v = _f(pc.get("roi_sharpness_min"))
            if v is not None and v < args.min_roi_sharpness:
                blurred.append(int(ci))
        if blurred:
            reasons.append("ROI 흐림 cam{}".format(blurred))

    return (not reasons), reasons


def camera_lag_report(captures: List[dict]) -> Dict[int, dict]:
    """캡처마다 가장 뒤처진 카메라를 세어 span 의 책임 소재를 드러낸다.

    span 은 "몇 ms 어긋났다"만 말해 주고 어느 장비가 원인인지는 말해 주지 않는다.
    같은 카메라가 반복해서 최하위면 그건 임계값 문제가 아니라 그 카메라 문제다.
    """
    worst = collections.Counter()
    lag_sum: Dict[int, float] = collections.defaultdict(float)
    n_used = 0
    for cap in captures:
        cams = cap.get("cams") or {}
        ts = {}
        for ci, rec in cams.items():
            t = _f(rec.get("host_monotonic_ts_ms"))
            if t is not None:
                ts[int(ci)] = t
        if len(ts) < 2:
            continue
        n_used += 1
        newest = max(ts.values())
        for ci, t in ts.items():
            lag_sum[ci] += newest - t
        worst[min(ts, key=lambda k: ts[k])] += 1
    return {ci: {"worst_count": worst.get(ci, 0),
                 "mean_lag_ms": lag_sum[ci] / n_used if n_used else 0.0}
            for ci in sorted(lag_sum)}


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", required=True,
                    help="캡처 폴더 (meta.json 이 있는 곳)")
    ap.add_argument("--out", default=None,
                    help="거른 meta 를 쓸 파일명 (--root 기준 상대경로). "
                         "생략하면 아무것도 쓰지 않고 통계만 낸다.")
    ap.add_argument("--max_span_ms", type=float, default=DEFAULT_MAX_SPAN_MS,
                    help="카메라 간 timestamp 최대 허용차. 0 이면 검사 안 함.")
    ap.add_argument("--min_fixed_pnp_ok", type=int, default=DEFAULT_MIN_FIXED_PNP_OK,
                    help="cube PnP 가 성공한 고정 카메라 최소 수")
    ap.add_argument("--max_pnp_reproj_px", type=float, default=DEFAULT_MAX_PNP_REPROJ_PX,
                    help="PnP 재투영 오차 상한. 0 이면 검사 안 함.")
    ap.add_argument("--max_roi_clip", type=float, default=DEFAULT_MAX_ROI_CLIP,
                    help="마커 ROI 포화 비율 상한(중앙값). 0 이면 검사 안 함.")
    ap.add_argument("--min_roi_sharpness", type=float, default=0.0,
                    help="마커 ROI Laplacian 분산 하한. 0 이면 검사 안 함(기본).")
    ap.add_argument("--sweep", action="store_true",
                    help="span 임계값을 바꿔가며 통과 수를 표로 보여준다")
    args = ap.parse_args()

    meta_path = os.path.join(args.root, "meta.json")
    if not os.path.exists(meta_path):
        sys.exit("[ERROR] meta.json 없음: {}".format(meta_path))
    with open(meta_path) as f:
        meta = json.load(f)
    caps = meta.get("captures") or []
    if not caps:
        sys.exit("[ERROR] captures 가 비어 있음")
    gripper_idx = meta.get("gripper_cam_idx")

    print("meta: {}  캡처 {}개  (gripper cam{})".format(meta_path, len(caps), gripper_idx))
    print("기준: span<={:.0f}ms  고정캠PnP>={}  reproj<={:.1f}px  ROI포화<={:.2f}"
          "{}".format(args.max_span_ms, args.min_fixed_pnp_ok, args.max_pnp_reproj_px,
                      args.max_roi_clip,
                      "  선명도>={:.0f}".format(args.min_roi_sharpness)
                      if args.min_roi_sharpness > 0 else ""))
    print()

    kept, dropped = [], []
    reason_count = collections.Counter()
    by_block = collections.Counter()
    for cap in caps:
        ok, reasons = evaluate(cap, gripper_idx, args)
        out = dict(cap)
        out["qc"] = {"pass": ok, "reasons": reasons}
        if ok:
            kept.append(out)
            by_block[cap.get("capture_block", "?")] += 1
        else:
            dropped.append(out)
            for r in reasons:
                # 수치를 지워 사유 종류별로 묶는다.
                reason_count[r.split(" ")[0]] += 1

    pct = 100.0 * len(kept) / len(caps)
    print("통과 {}/{} ({:.0f}%)".format(len(kept), len(caps), pct))
    for blk, n in sorted(by_block.items()):
        print("   {:<14} {}".format(blk, n))
    if reason_count:
        print("\n탈락 사유(중복 포함):")
        for r, n in reason_count.most_common():
            print("   {:<14} {}".format(r, n))

    lag = camera_lag_report(caps)
    if lag:
        print("\n카메라별 지연 (최신 프레임 대비):")
        print("   {:<6} {:>12} {:>14}".format("cam", "평균지연(ms)", "최하위 횟수"))
        for ci, st in sorted(lag.items(), key=lambda kv: -kv[1]["mean_lag_ms"]):
            tag = "  <- span 주범" if st["worst_count"] > len(caps) * 0.4 else ""
            print("   cam{:<3} {:12.1f} {:14d}{}".format(
                ci, st["mean_lag_ms"], st["worst_count"], tag))

    if args.sweep:
        print("\nspan 임계값별 통과 수:")
        base = args.max_span_ms
        for thr in (0, 120, 250, 500, 1000, 2000, 5000):
            args.max_span_ms = thr
            n = sum(1 for c in caps if evaluate(c, gripper_idx, args)[0])
            label = "검사안함" if thr == 0 else "<= {:>5}ms".format(thr)
            print("   {:<12} {:3d}/{} ({:.0f}%)".format(
                label, n, len(caps), 100.0 * n / len(caps)))
        args.max_span_ms = base

    if args.out is None:
        print("\n(--out 미지정: 파일 안 씀)")
        return 0

    out_path = os.path.join(args.root, args.out)
    if os.path.realpath(out_path) == os.path.realpath(meta_path):
        sys.exit("[ERROR] 원본 meta.json 을 덮어쓸 수 없다. 다른 이름을 줄 것.")
    out_meta = dict(meta)
    out_meta["captures"] = kept
    out_meta["qc_filter"] = {
        "source_meta": os.path.abspath(meta_path),
        "n_input": len(caps),
        "n_kept": len(kept),
        "thresholds": {
            "max_span_ms": args.max_span_ms,
            "min_fixed_pnp_ok": args.min_fixed_pnp_ok,
            "max_pnp_reproj_px": args.max_pnp_reproj_px,
            "max_roi_clip": args.max_roi_clip,
            "min_roi_sharpness": args.min_roi_sharpness,
        },
        "note": "Step2b_filter_captures.py 산출물. 이미지 파일은 삭제하지 않는다.",
    }
    with open(out_path, "w") as f:
        json.dump(out_meta, f, indent=2)
    print("\n[ok] {} ({}개 캡처)".format(out_path, len(kept)))
    print("     이미지는 그대로 둔다. Step3 에 이 meta 를 쓰면 걸러진 것만 들어간다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
