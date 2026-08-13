"""Step3-compatible FK set-prior alignment for the synthetic experiments.

The functions in this module intentionally mirror the numerical policy used by
``Step3_calibration.py``: weighted chordal SE(3) averaging, iterative MAD
rejection, right-multiplied set-prior alignment, and guarded prior blending.
Keeping the implementation local preserves ``Simulation`` as a standalone
package while making the correspondence testable.
"""
from dataclasses import dataclass
from typing import Dict, Mapping, Optional

import numpy as np

from .se3 import inv_T, rot_deg


def _se3_distance(Ta: np.ndarray, Tb: np.ndarray) -> float:
    """Same distance used by ``utils_pose.se3_distance`` in Step3."""
    dR = Ta[:3, :3] @ Tb[:3, :3].T
    cos_angle = np.clip((np.trace(dR) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(cos_angle) + np.linalg.norm(Ta[:3, 3] - Tb[:3, 3]))


def weighted_se3_average(T_list, w_list=None) -> np.ndarray:
    """Exact standalone counterpart of Step3 ``weighted_se3_average``."""
    if len(T_list) == 0:
        raise ValueError("T_list is empty")
    if w_list is None:
        w = np.ones((len(T_list),), dtype=np.float64)
    else:
        w = np.asarray(w_list, dtype=np.float64)
        w = np.maximum(w, 1e-12)
    w = w / (w.sum() + 1e-12)

    ts = np.asarray([T[:3, 3] for T in T_list], dtype=np.float64)
    Rs = np.asarray([T[:3, :3] for T in T_list], dtype=np.float64)
    t_mean = (w[:, None] * ts).sum(axis=0)
    R_mean = (w[:, None, None] * Rs).sum(axis=0)
    U, _, Vt = np.linalg.svd(R_mean)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt

    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = R
    out[:3, 3] = t_mean
    return out


def robust_weighted_se3_average(T_list, w_list=None, k_mad=2.5,
                                 max_iters=3, return_stats=False):
    """Exact standalone counterpart of Step3's robust weighted average."""
    if len(T_list) == 0:
        raise ValueError("T_list is empty")
    if w_list is None:
        w = np.ones((len(T_list),), dtype=np.float64)
    else:
        w = np.asarray(w_list, dtype=np.float64)
        w = np.maximum(w, 1e-12)

    idx = np.arange(len(T_list), dtype=int)
    T_curr = weighted_se3_average(T_list, w)
    for _ in range(max_iters):
        res = np.asarray([_se3_distance(T_list[i], T_curr) for i in idx])
        med = np.median(res)
        mad = np.median(np.abs(res - med)) + 1e-12
        keep = res <= med + k_mad * 1.4826 * mad
        if keep.sum() < max(3, int(0.35 * len(idx))):
            break
        new_idx = idx[keep]
        if len(new_idx) == len(idx):
            break
        idx = new_idx
        T_curr = weighted_se3_average(
            [T_list[i] for i in idx], [w[i] for i in idx])

    T_final = weighted_se3_average(
        [T_list[i] for i in idx], [w[i] for i in idx])
    if not return_stats:
        return T_final

    trans = [np.linalg.norm(T_list[i][:3, 3] - T_final[:3, 3]) * 1000.0
             for i in idx]
    rotation = [rot_deg(T_list[i], T_final) for i in idx]
    stats = {
        "num_frames": int(len(T_list)),
        "num_inliers": int(len(idx)),
        "inlier_ratio": float(len(idx) / max(len(T_list), 1)),
        "translation_std_mm": float(np.std(trans)) if trans else 0.0,
        "rotation_std_deg": float(np.std(rotation)) if rotation else 0.0,
    }
    return T_final, stats


def blend_rigid_transforms(T_a: np.ndarray, T_b: np.ndarray,
                           alpha: float) -> np.ndarray:
    """Step3's translation interpolation plus projected rotation blend."""
    alpha = float(np.clip(alpha, 0.0, 1.0))
    if alpha <= 0.0:
        return np.asarray(T_a, dtype=np.float64).copy()
    if alpha >= 1.0:
        return np.asarray(T_b, dtype=np.float64).copy()
    T_a = np.asarray(T_a, dtype=np.float64)
    T_b = np.asarray(T_b, dtype=np.float64)
    out = np.eye(4, dtype=np.float64)
    out[:3, 3] = (1.0 - alpha) * T_a[:3, 3] + alpha * T_b[:3, 3]
    R_mean = (1.0 - alpha) * T_a[:3, :3] + alpha * T_b[:3, :3]
    U, _, Vt = np.linalg.svd(R_mean)
    out[:3, :3] = U @ Vt
    if np.linalg.det(out[:3, :3]) < 0:
        U[:, -1] *= -1
        out[:3, :3] = U @ Vt
    return out


@dataclass(frozen=True)
class PriorAlignmentResult:
    corrected: Dict[int, np.ndarray]
    anchors: Dict[int, np.ndarray]
    delta: Optional[np.ndarray]
    diagnostics: dict


def adaptive_gate_threshold(values, k: float, floor: float = 0.0,
                            min_sets: int = 5) -> Optional[float]:
    """Median + k*1.4826*MAD cutoff over per-set gate distances.

    d_t and d_R measure how far one set's delta sits from the common delta, so
    their scale is a property of the data, not a fixed physical tolerance.

    ``floor`` guards the degenerate case where the sets happen to agree so
    closely that MAD collapses to zero and the cutoff lands on the median,
    which would reject half of a perfectly healthy batch. Callers pass vision's
    own scatter, below which a FK deviation is indistinguishable from noise.

    Returns None when there are too few sets for the median/MAD to mean
    anything; callers then fall back to the fixed thresholds.

    Floor definition, shared with ``Step3_calibration.resolve_prior_gate``:
    the scatter of the observations that formed each set's consensus, taken
    at its median across sets. Threshold arithmetic parity with Step3 is
    covered by ``tests/test_production_alignment.py``.
    """
    vals = np.asarray([v for v in values if v is not None], dtype=np.float64)
    if vals.size < min_sets:
        return None
    med = float(np.median(vals))
    mad = float(np.median(np.abs(vals - med)))
    return max(med + k * 1.4826 * mad, float(floor))


def align_and_blend_set_priors(
        raw_priors: Mapping[int, np.ndarray],
        visual_by_set: Mapping[int, np.ndarray],
        support_by_set: Optional[Mapping[int, float]] = None,
        max_prior_dt_mm: float = 35.0,
        max_prior_dr_deg: float = 8.0,
        gate_mode: str = "adaptive",
        gate_k: float = 2.5,
        scatter_by_set: Optional[Mapping[int, tuple]] = None,
        gate_min_sets: int = 5) -> PriorAlignmentResult:
    """Apply the complete Step3 set-prior alignment and guarded blend policy.

    ``corrected`` is raw FK right-multiplied by the robust common delta.
    ``anchors`` is the corrected prior for every set the gate accepts, and the
    visual estimate for the rest. A set that clears the gate is trusted fully;
    there is no partial-trust weight.

    ``gate_mode`` selects how the accept threshold is chosen:
      * ``"fixed"``    - the original constant 35 mm / 8 deg limits.
      * ``"adaptive"`` - median + k*1.4826*MAD over the per-set distances,
        floored by vision's own scatter (``scatter_by_set``) so a near-zero MAD
        cannot reject sets whose FK deviation is smaller than the resolution of
        the estimate it is compared against.
    """
    support_by_set = support_by_set or {}
    common = sorted(set(raw_priors) & set(visual_by_set))
    if not common:
        return PriorAlignmentResult({}, dict(visual_by_set), None, {
            "support": 0, "stability": {}, "per_set": {}})

    deltas = [inv_T(np.asarray(raw_priors[s])) @ np.asarray(visual_by_set[s])
              for s in common]
    weights = [max(float(support_by_set.get(s, 1.0)), 1.0) for s in common]
    delta, stability = robust_weighted_se3_average(
        deltas, weights, return_stats=True)
    corrected = {int(s): np.asarray(T).copy() @ delta
                 for s, T in raw_priors.items()}

    dist = {}
    for s, visual in visual_by_set.items():
        prior = corrected.get(int(s))
        if prior is None:
            continue
        dist[int(s)] = (
            float(np.linalg.norm(np.asarray(visual)[:3, 3] - prior[:3, 3]) * 1000.0),
            rot_deg(np.asarray(visual), prior),
        )

    floor_dt = floor_dr = 0.0
    if scatter_by_set:
        floor_dt = float(np.median([v[0] for v in scatter_by_set.values()]))
        floor_dr = float(np.median([v[1] for v in scatter_by_set.values()]))

    gate_dt, gate_dr = float(max_prior_dt_mm), float(max_prior_dr_deg)
    if gate_mode == "adaptive" and dist:
        adaptive_dt = adaptive_gate_threshold(
            [v[0] for v in dist.values()], gate_k, floor_dt, gate_min_sets)
        adaptive_dr = adaptive_gate_threshold(
            [v[1] for v in dist.values()], gate_k, floor_dr, gate_min_sets)
        if adaptive_dt is not None:
            gate_dt = adaptive_dt
        if adaptive_dr is not None:
            gate_dr = adaptive_dr

    anchors = {}
    per_set = {}
    for s, visual in visual_by_set.items():
        visual = np.asarray(visual)
        prior = corrected.get(int(s))
        accepted = False
        dt_mm = dr_deg = None
        anchor = visual.copy()
        if prior is not None:
            dt_mm, dr_deg = dist[int(s)]
            accepted = dt_mm <= gate_dt and dr_deg <= gate_dr
            if accepted:
                anchor = prior.copy()
        anchors[int(s)] = anchor
        per_set[str(int(s))] = {
            "support": int(support_by_set.get(s, 1)),
            "prior_accepted": bool(accepted),
            "prior_blend_dt_mm": dt_mm,
            "prior_blend_dr_deg": dr_deg,
            # 그 세트의 vision 합의가 얼마나 흔들리는지(관측들의 산포).
            # 상수 없는 가중치 계산에 쓰인다.
            "vision_scatter_mm": (float(scatter_by_set.get(int(s), (0.0, 0.0))[0])
                                  if scatter_by_set else None),
            "vision_scatter_deg": (float(scatter_by_set.get(int(s), (0.0, 0.0))[1])
                                   if scatter_by_set else None),
        }

    return PriorAlignmentResult(corrected, anchors, delta, {
        "support": int(len(common)),
        "stability": stability,
        "gate_mode": str(gate_mode),
        "gate_dt_mm": float(gate_dt),
        "gate_dr_deg": float(gate_dr),
        "gate_floor_dt_mm": float(floor_dt),
        "gate_floor_dr_deg": float(floor_dr),
        "max_prior_dt_mm": float(max_prior_dt_mm),
        "max_prior_dr_deg": float(max_prior_dr_deg),
        "per_set": per_set,
    })
