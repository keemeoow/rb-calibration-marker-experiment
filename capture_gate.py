"""Block-aware capture acceptance rules with no camera/runtime dependencies.

The acquisition program imports this module, while unit tests can exercise the
gate without importing OpenCV, RealSense, or robot libraries.
"""

from typing import Dict, Optional, Tuple


CAPTURE_BLOCK_A = "A_placement"
CAPTURE_BLOCK_B = "B_eyetohand"
KNOWN_CAPTURE_BLOCKS = (CAPTURE_BLOCK_A, CAPTURE_BLOCK_B)


def resolve_capture_gate_profile(gate_cfg: dict,
                                 capture_block: Optional[str]) -> Tuple[str, dict, bool]:
    """Return ``(block, profile, known)`` and keep legacy flat configs usable."""
    block = CAPTURE_BLOCK_A if capture_block is None else str(capture_block)
    profiles = gate_cfg.get("profiles") if isinstance(gate_cfg, dict) else None
    if isinstance(profiles, dict):
        profile = profiles.get(block)
        if isinstance(profile, dict):
            return block, profile, block in KNOWN_CAPTURE_BLOCKS
        return block, {}, False
    # A flat config predates block-specific gates and is treated as A only.
    return block, gate_cfg if isinstance(gate_cfg, dict) else {}, block == CAPTURE_BLOCK_A


def evaluate_capture_gate(frames_dict: Dict[int, dict],
                          gate_cfg: dict,
                          gripper_cam_idx: Optional[int] = None,
                          capture_block: Optional[str] = None,
                          cube_gripped: Optional[bool] = None) -> dict:
    block, profile, known_block = resolve_capture_gate_profile(gate_cfg, capture_block)
    cams_with_cube = 0
    fixed_visible = 0
    fixed_multimarker_cams = 0
    cube_pnp_ok_cams = 0
    fixed_cube_pnp_ok_cams = 0
    depth_valid_cams = 0
    fixed_depth_valid_cams = 0
    fixed_depth_quality_cams = 0
    missing_timestamp_cams = 0
    capture_ts = []
    per_camera = {}
    gripper_markers = 0
    gripper_charuco_corners = 0
    gripper_cube_pnp_ok = False
    gripper_depth_valid = False
    gripper_depth_plane_mean_mm = None

    fixed_multimarker_min_markers = int(profile.get("fixed_multimarker_min_markers", 2))
    max_fixed_depth_plane_mean_mm = float(profile.get("max_fixed_depth_plane_mean_mm", 0.0))
    max_cube_pnp_reproj_mean_px = float(profile.get("max_cube_pnp_reproj_mean_px", 0.0))
    min_depth_samples = int(profile.get("min_depth_samples", 0))

    for ci, fr in frames_dict.items():
        is_gripper = gripper_cam_idx is not None and int(ci) == int(gripper_cam_idx)
        n_markers = int(fr.get("n_markers", 0))
        cube_visible = bool(fr.get("ok", False))
        cube_pnp = fr.get("cube_pnp")
        cube_pnp_solved = bool(cube_pnp is not None)
        cube_pnp_reproj_mean_px = None if not cube_pnp else cube_pnp.get("reproj_mean_px")
        cube_pnp_ok = bool(
            cube_pnp_solved
            and (
                max_cube_pnp_reproj_mean_px <= 0
                or (
                    cube_pnp_reproj_mean_px is not None
                    and float(cube_pnp_reproj_mean_px) <= max_cube_pnp_reproj_mean_px
                )
            )
        )
        depth_num_samples = 0 if not cube_pnp else int(cube_pnp.get("depth_num_samples", 0))
        depth_valid = bool(
            cube_pnp_ok
            and cube_pnp.get("depth_valid")
            and depth_num_samples >= min_depth_samples
        )
        depth_plane_mean_mm = None if not cube_pnp else cube_pnp.get("depth_plane_mean_mm")
        fixed_depth_quality = bool(
            not is_gripper
            and cube_pnp_ok
            and depth_valid
            and (
                max_fixed_depth_plane_mean_mm <= 0
                or (
                    depth_plane_mean_mm is not None
                    and float(depth_plane_mean_mm) <= max_fixed_depth_plane_mean_mm
                )
            )
        )
        ts_ms = fr.get("ts_ms")
        if ts_ms is not None:
            capture_ts.append(float(ts_ms))
        else:
            missing_timestamp_cams += 1
        if cube_visible:
            cams_with_cube += 1
            if not is_gripper:
                fixed_visible += 1
                if n_markers >= fixed_multimarker_min_markers:
                    fixed_multimarker_cams += 1
        if cube_pnp_ok:
            cube_pnp_ok_cams += 1
            if not is_gripper:
                fixed_cube_pnp_ok_cams += 1
        if depth_valid:
            depth_valid_cams += 1
            if not is_gripper:
                fixed_depth_valid_cams += 1
        if fixed_depth_quality:
            fixed_depth_quality_cams += 1
        if is_gripper:
            gripper_markers = int(n_markers)
            ch_ids = fr.get("ch_ids")
            gripper_charuco_corners = 0 if ch_ids is None else len(ch_ids)
            gripper_cube_pnp_ok = cube_pnp_ok
            gripper_depth_valid = depth_valid
            if depth_plane_mean_mm is not None:
                gripper_depth_plane_mean_mm = float(depth_plane_mean_mm)
        per_camera[int(ci)] = {
            "n_markers": n_markers,
            "cube_visible": cube_visible,
            "cube_pnp_solved": cube_pnp_solved,
            "cube_pnp_ok": cube_pnp_ok,
            "cube_pnp_reproj_mean_px": cube_pnp_reproj_mean_px,
            "depth_valid": depth_valid,
            "depth_num_samples": depth_num_samples,
            "fixed_depth_quality": fixed_depth_quality,
        }

    capture_span_ms = (max(capture_ts) - min(capture_ts)) if len(capture_ts) >= 2 else 0.0
    reasons = []
    min_cams_with_cube = int(profile.get("min_cams_with_cube", 0))
    min_fixed_cams_with_cube = int(profile.get("min_fixed_cams_with_cube", 0))
    min_fixed_multimarker_cams = int(profile.get("min_fixed_multimarker_cams", 0))
    min_cube_pnp_ok_cams = int(profile.get("min_cube_pnp_ok_cams", 0))
    min_fixed_cube_pnp_ok_cams = int(profile.get("min_fixed_cube_pnp_ok_cams", 0))
    min_fixed_depth_quality_cams = int(profile.get("min_fixed_depth_quality_cams", 0))
    min_gripper_markers = int(profile.get("min_gripper_markers", 0))
    min_gripper_charuco_corners = int(profile.get("min_gripper_charuco_corners", 0))
    require_gripper_cube_pnp = bool(profile.get("require_gripper_cube_pnp", False))
    require_gripper_depth_valid = bool(profile.get("require_gripper_depth_valid", False))
    max_gripper_depth_plane_mean_mm = float(profile.get("max_gripper_depth_plane_mean_mm", 0.0))
    max_capture_span_ms = float(profile.get("max_capture_span_ms", 0.0))
    require_all_frame_timestamps = bool(profile.get("require_all_frame_timestamps", True))
    expected_cube_gripped = profile.get("expected_cube_gripped")

    if not known_block:
        reasons.append("unknown capture_block '{}'".format(block))
    if expected_cube_gripped is not None and cube_gripped is not None:
        if bool(cube_gripped) != bool(expected_cube_gripped):
            reasons.append(
                "capture_block {} requires cube_gripped={}".format(
                    block, bool(expected_cube_gripped)
                )
            )
    if cams_with_cube < min_cams_with_cube:
        reasons.append("cube-visible cams {} < required {}".format(cams_with_cube, min_cams_with_cube))
    if fixed_visible < min_fixed_cams_with_cube:
        reasons.append(
            "fixed cube-visible cams {} < required {}".format(fixed_visible, min_fixed_cams_with_cube)
        )
    if fixed_multimarker_cams < min_fixed_multimarker_cams:
        reasons.append(
            "fixed multi-marker cams {} < required {} (markers/cam >= {})".format(
                fixed_multimarker_cams, min_fixed_multimarker_cams, fixed_multimarker_min_markers
            )
        )
    if cube_pnp_ok_cams < min_cube_pnp_ok_cams:
        reasons.append("cube_pnp-ok cams {} < required {}".format(cube_pnp_ok_cams, min_cube_pnp_ok_cams))
    if fixed_cube_pnp_ok_cams < min_fixed_cube_pnp_ok_cams:
        reasons.append(
            "fixed cube_pnp-ok cams {} < required {}".format(
                fixed_cube_pnp_ok_cams, min_fixed_cube_pnp_ok_cams
            )
        )
    if fixed_depth_quality_cams < min_fixed_depth_quality_cams:
        reasons.append(
            "fixed depth-quality cams {} < required {}".format(
                fixed_depth_quality_cams, min_fixed_depth_quality_cams
            )
        )
    if require_all_frame_timestamps and missing_timestamp_cams > 0:
        reasons.append("missing timestamps for {} camera frame(s)".format(missing_timestamp_cams))
    if min_gripper_markers > 0 and gripper_markers < min_gripper_markers:
        reasons.append(
            "gripper cube markers {} < required {}".format(gripper_markers, min_gripper_markers)
        )
    if require_gripper_cube_pnp and not gripper_cube_pnp_ok:
        reasons.append("gripper cube_pnp missing")
    if min_gripper_charuco_corners > 0 and gripper_charuco_corners < min_gripper_charuco_corners:
        reasons.append(
            "gripper charuco corners {} < required {}".format(
                gripper_charuco_corners, min_gripper_charuco_corners
            )
        )
    if require_gripper_depth_valid and require_gripper_cube_pnp and gripper_cube_pnp_ok and not gripper_depth_valid:
        reasons.append("gripper depth support invalid")
    if (
        require_gripper_depth_valid
        and gripper_depth_valid
        and max_gripper_depth_plane_mean_mm > 0
        and gripper_depth_plane_mean_mm is None
    ):
        reasons.append("gripper depth plane metric missing")
    if (
        require_gripper_depth_valid
        and gripper_depth_valid
        and max_gripper_depth_plane_mean_mm > 0
        and gripper_depth_plane_mean_mm is not None
        and float(gripper_depth_plane_mean_mm) > max_gripper_depth_plane_mean_mm
    ):
        reasons.append(
            "gripper depth plane {:.1f}mm > {:.1f}mm".format(
                float(gripper_depth_plane_mean_mm), max_gripper_depth_plane_mean_mm
            )
        )
    if max_capture_span_ms > 0 and capture_span_ms > max_capture_span_ms:
        reasons.append(
            "timestamp span {:.1f}ms > {:.1f}ms".format(capture_span_ms, max_capture_span_ms)
        )

    return {
        "pass": bool(not reasons),
        "status": "PASS" if not reasons else "FAIL",
        "reason": "capture gate satisfied" if not reasons else " | ".join(reasons),
        "reasons": reasons,
        "capture_block": block,
        "gate_profile": block,
        "cube_gripped": cube_gripped,
        "expected_cube_gripped": expected_cube_gripped,
        "cams_with_cube": int(cams_with_cube),
        "min_cams_with_cube": int(min_cams_with_cube),
        "capture_span_ms": float(capture_span_ms),
        "max_capture_span_ms": float(max_capture_span_ms),
        "missing_timestamp_cams": int(missing_timestamp_cams),
        "require_all_frame_timestamps": bool(require_all_frame_timestamps),
        "fixed_visible_cams": int(fixed_visible),
        "min_fixed_cams_with_cube": int(min_fixed_cams_with_cube),
        "fixed_multimarker_cams": int(fixed_multimarker_cams),
        "min_fixed_multimarker_cams": int(min_fixed_multimarker_cams),
        "fixed_multimarker_min_markers": int(fixed_multimarker_min_markers),
        "max_cube_pnp_reproj_mean_px": float(max_cube_pnp_reproj_mean_px),
        "min_depth_samples": int(min_depth_samples),
        "cube_pnp_ok_cams": int(cube_pnp_ok_cams),
        "min_cube_pnp_ok_cams": int(min_cube_pnp_ok_cams),
        "fixed_cube_pnp_ok_cams": int(fixed_cube_pnp_ok_cams),
        "min_fixed_cube_pnp_ok_cams": int(min_fixed_cube_pnp_ok_cams),
        "depth_valid_cams": int(depth_valid_cams),
        "fixed_depth_valid_cams": int(fixed_depth_valid_cams),
        "fixed_depth_quality_cams": int(fixed_depth_quality_cams),
        "min_fixed_depth_quality_cams": int(min_fixed_depth_quality_cams),
        "max_fixed_depth_plane_mean_mm": float(max_fixed_depth_plane_mean_mm),
        "gripper_markers": int(gripper_markers),
        "min_gripper_markers": int(min_gripper_markers),
        "gripper_charuco_corners": int(gripper_charuco_corners),
        "min_gripper_charuco_corners": int(min_gripper_charuco_corners),
        "gripper_cube_pnp_ok": bool(gripper_cube_pnp_ok),
        "gripper_depth_valid": bool(gripper_depth_valid),
        "gripper_depth_plane_mean_mm": gripper_depth_plane_mean_mm,
        "per_camera": per_camera,
    }
