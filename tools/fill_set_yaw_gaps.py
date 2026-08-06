#!/usr/bin/env python3
"""Fill yaw gaps in a taught cube-placement pool without going back to the robot.

`test_robot_teaching_waypoint.py` requires the placement sets to cover yaw with
range >= 300 deg and no gap > 60 deg. Hand-teaching extra placements just to
close a gap is slow and, worse, moves the cube to a new XY that then has to be
re-measured.

There is an exact shortcut. The tool frame (`settool(3|4, 0, 0, z, 0, 0, 0)`) is
a pure translation along the J6 axis, so rotating joint 6 alone leaves the TCP
*position* untouched and only spins the cube about the vertical. Concretely the
pool obeys

    rz = d1 - d6 + k   (mod 360)

with k fitted from the taught sets. So a placement at any desired yaw is a clone
of an existing set with d6 shifted by -(delta yaw) -- no IK, no re-teach, and the
cube stays at an XY the operator already validated as reachable and visible.

The synthesised set carries the clone's place_tcp/cube_center position with only
rz rewritten. Those values are a fallback anyway: `_run_auto_multiset` re-measures
`set_cube_center_6dof` with tool4 after physically placing the cube.

    # inspect the gaps, write nothing
    python tools/fill_set_yaw_gaps.py --sets data/session01/capture_sets_001.json

    # close them and write the augmented pool
    python tools/fill_set_yaw_gaps.py --sets data/session01/capture_sets_001.json \
        --output data/session02/capture_sets.json
"""
import argparse
import json
import os
import sys

import numpy as np

MAX_GAP_DEG = 60.0     # test_robot_teaching_waypoint.py set threshold
MIN_RANGE_DEG = 300.0  # ditto


def _yaw(s):
    return float(s["place_tcp"][3]) % 360.0


def fit_k(sets):
    """k in rz = d1 - d6 + k, averaged over the taught sets (circular mean)."""
    ks = []
    for s in sets:
        rz = float(s["place_tcp"][3])
        d1, d6 = float(s["place_joints"][0]), float(s["place_joints"][5])
        ks.append(np.deg2rad((rz - (d1 - d6)) % 360.0))
    mean = np.rad2deg(np.arctan2(np.mean(np.sin(ks)), np.mean(np.cos(ks)))) % 360.0
    spread = float(np.rad2deg(np.std(np.unwrap(ks))))
    return float(mean), spread


def gaps(sets):
    """(sorted yaws, gap to the next yaw going counter-clockwise)."""
    y = sorted(_yaw(s) for s in sets)
    return y, [(y[(i + 1) % len(y)] - y[i]) % 360.0 for i in range(len(y))]


def plan_fills(sets, max_gap=MAX_GAP_DEG):
    """Yaw targets that bring every gap under ``max_gap``, fewest sets first."""
    y, g = gaps(sets)
    targets = []
    for i, gap in enumerate(g):
        if gap <= max_gap:
            continue
        n = int(np.ceil(gap / max_gap)) - 1
        for j in range(1, n + 1):
            targets.append((y[i] + gap * j / (n + 1)) % 360.0)
    return targets


def clone_at_yaw(sets, target_yaw, k):
    """Clone the set whose yaw is nearest ``target_yaw`` and spin d6 to reach it.

    Nearest is the right choice: the smaller the d6 shift, the less likely it
    walks joint 6 into a limit or a wrap the controller resolves differently.
    """
    diffs = [abs(((_yaw(s) - target_yaw + 180.0) % 360.0) - 180.0) for s in sets]
    src = sets[int(np.argmin(diffs))]
    delta = ((target_yaw - _yaw(src) + 180.0) % 360.0) - 180.0

    out = json.loads(json.dumps(src))  # deep copy of plain JSON
    out["place_joints"] = list(src["place_joints"])
    out["place_joints"][5] = float(src["place_joints"][5]) - delta
    for key in ("place_tcp", "set_cube_center_6dof"):
        if key in out:
            out[key] = list(src[key])
            out[key][3] = float(((target_yaw + 180.0) % 360.0) - 180.0)
    out["derived_from_set_index"] = int(src["set_index"])
    out["derived_by"] = "d6_yaw_fill"
    out["derived_d6_delta_deg"] = float(-delta)
    return out, src, delta


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sets", required=True, help="taught capture_sets JSON")
    ap.add_argument("--output", default=None,
                    help="write the augmented pool here (omit to only report)")
    ap.add_argument("--max_gap_deg", type=float, default=MAX_GAP_DEG)
    ap.add_argument("--d6_limit_deg", type=float, nargs=2, default=None,
                    metavar=("LO", "HI"),
                    help="reject fills whose d6 leaves this range (default: the "
                         "range spanned by the taught sets, padded 20 deg)")
    args = ap.parse_args()

    with open(args.sets) as f:
        data = json.load(f)
    sets = data.get("capture_sets")
    if not isinstance(sets, list) or len(sets) < 2:
        sys.exit("[ERROR] capture_sets must hold at least 2 taught sets")

    k, spread = fit_k(sets)
    y, g = gaps(sets)
    print("taught sets: {}   k = {:.2f} deg (spread {:.2f})".format(len(sets), k, spread))
    if spread > 5.0:
        print("  [!] k is not consistent across sets. The pool may mix tool frames "
              "or wrapped joints; verify before trusting a synthesised placement.")
    print("  yaw: {}".format([round(v) for v in y]))
    print("  gap: {}   max={:.0f} range={:.0f}".format(
        [round(v) for v in g], max(g), 360.0 - max(g)))

    if args.d6_limit_deg is not None:
        lo, hi = args.d6_limit_deg
    else:
        d6s = [float(s["place_joints"][5]) for s in sets]
        lo, hi = min(d6s) - 20.0, max(d6s) + 20.0
    print("  d6 allowed: [{:.1f}, {:.1f}]".format(lo, hi))

    targets = plan_fills(sets, args.max_gap_deg)
    if not targets:
        print("\nEvery gap is already <= {:.0f} deg. Nothing to fill.".format(args.max_gap_deg))
        return 0

    print("\nfills needed: {}".format(len(targets)))
    pool = list(sets)
    added = []
    for t in targets:
        new, src, delta = clone_at_yaw(pool, t, k)
        d6 = new["place_joints"][5]
        flag = "" if lo <= d6 <= hi else "   <-- d6 OUT OF RANGE, skipped"
        print("  yaw {:6.1f}  <- clone set#{} (yaw {:6.1f}), d6 {:.2f} -> {:.2f}{}".format(
            t, src["set_index"], _yaw(src), float(src["place_joints"][5]), d6, flag))
        if flag:
            continue
        new["set_index"] = len(pool)
        pool.append(new)
        added.append(new)

    if not added:
        sys.exit("\n[ERROR] every candidate fill left the d6 range. Teach these "
                 "placements by hand, or widen --d6_limit_deg if the limit is wrong.")

    y2, g2 = gaps(pool)
    ok = max(g2) <= args.max_gap_deg and (360.0 - max(g2)) >= MIN_RANGE_DEG
    print("\nafter fill: {} sets   max gap={:.0f} (<= {:.0f})   range={:.0f} (>= {:.0f})   {}".format(
        len(pool), max(g2), args.max_gap_deg, 360.0 - max(g2), MIN_RANGE_DEG,
        "PASS" if ok else "STILL FAILING"))

    if args.output is None:
        print("\n(--output not given; nothing written)")
        return 0 if ok else 1

    os.makedirs(os.path.dirname(os.path.abspath(args.output)) or ".", exist_ok=True)
    out = dict(data)
    out["capture_sets"] = pool
    out["yaw_fill"] = {
        "source": os.path.abspath(args.sets),
        "k_deg": k,
        "n_taught": len(sets),
        "n_synthesised": len(added),
        "method": "clone nearest set, rotate d6 only (TCP position invariant)",
    }
    with open(args.output, "w") as f:
        json.dump(out, f, indent=2)
    print("[ok] wrote {}".format(args.output))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
