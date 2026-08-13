"""Canonical method schema and comparison contract for Table 1.

Robot FK ``T_base_gripper`` is a common kinematic backbone in every condition.
The pose-source fields below describe only whether a target pose is fixed from
that FK backbone or estimated from images; they never mean "use FK at all".
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import re
from typing import Iterable, Mapping


TARGET_SETS = {"board", "cube", "cube+board"}
UNIFIED_MODES = {"seq", "U"}
POSE_SOURCE_ESTIMATED = "estimated"
POSE_SOURCE_FK_FIXED = "FK-fixed"
POSE_SOURCE_ABSENT = "—"
_SOFT_ANCHOR_RE = re.compile(r"soft-anchor \(λ=(?:0|[1-9][0-9]*)(?:\.[0-9]+)?\)")


@dataclass(frozen=True)
class AblationCondition:
    row: str
    target_set: str
    unified: str
    fk_to_cube: str
    fk_to_board: str
    label: str
    supplementary: bool = False


MAIN_ABLATION_CONDITIONS = (
    AblationCondition("A0", "board", "seq", "—", "estimated", "baseline"),
    AblationCondition("A1", "cube+board", "seq", "estimated", "estimated", "+cube"),
    AblationCondition("A2", "cube+board", "U", "estimated", "estimated", "+unified"),
    AblationCondition("A3", "cube+board", "U", "FK-fixed", "estimated", "Ours (full)"),
    AblationCondition("B1", "cube+board", "seq", "FK-fixed", "estimated", "−Unified"),
    AblationCondition("B2", "cube", "U", "FK-fixed", "—", "−board"),
    AblationCondition("B3", "board", "U", "—", "estimated", "−cube"),
)

SUPPLEMENTARY_CONDITIONS = (
    AblationCondition(
        "C1",
        "cube+board",
        "U",
        "soft-anchor (λ=5)",
        "estimated",
        "FK-weight sensitivity",
        supplementary=True,
    ),
)

# Inputs that are constants, never optimization variables.  In particular,
# intrinsics must not leak into stage 2 as an accidental extra degree of
# freedom.  The observation mask is computed once, before any row is fit.
GLOBAL_FIXED_INPUTS = (
    "camera_intrinsics_K",
    "distortion_D",
    "target_geometry",
    "T_base_gripper_by_event",
    "train_test_split",
    "observation_quality_mask",
    "optimizer_loss_tolerances",
    "initialization_seeds",
)


@dataclass(frozen=True)
class SequentialStageSpec:
    stage1_observations: str
    stage1_free: tuple[str, ...]
    stage1_fixed: tuple[str, ...]
    stage2_observations: str
    stage2_free: tuple[str, ...]
    stage2_frozen_from_stage1: tuple[str, ...]


# Stage 2 may optimize only the fixed-camera extrinsics.  It must never update
# hand-eye or target poses, and there is no alternating pass back to stage 1.
SEQUENTIAL_STAGE_SPECS: Mapping[str, SequentialStageSpec] = {
    "A0": SequentialStageSpec(
        "eih_only",
        ("T_gripper_cam", "T_base_board"),
        GLOBAL_FIXED_INPUTS,
        "e2h_only",
        ("T_base_Ci",),
        ("T_gripper_cam", "T_base_board"),
    ),
    "A1": SequentialStageSpec(
        "eih_only",
        ("T_gripper_cam", "T_base_board", "T_base_cube_by_set"),
        GLOBAL_FIXED_INPUTS,
        "e2h_only",
        ("T_base_Ci",),
        ("T_gripper_cam", "T_base_board", "T_base_cube_by_set"),
    ),
    "B1": SequentialStageSpec(
        "eih_only",
        ("T_gripper_cam", "T_base_board"),
        GLOBAL_FIXED_INPUTS + ("shared_board_free_FK_cube_poses",),
        "e2h_only",
        ("T_base_Ci",),
        ("T_gripper_cam", "T_base_board", "shared_board_free_FK_cube_poses"),
    ),
}

# Joint rows use both camera paths in one objective.  These lists are the only
# permitted optimization variables for the corresponding rows.
UNIFIED_FREE_VARIABLES: Mapping[str, tuple[str, ...]] = {
    "A2": ("T_base_Ci", "T_gripper_cam", "T_base_board", "T_base_cube_by_set"),
    "A3": ("T_base_Ci", "T_gripper_cam", "T_base_board"),
    "B2": ("T_base_Ci", "T_gripper_cam"),
    "B3": ("T_base_Ci", "T_gripper_cam", "T_base_board"),
}

# This artifact is prepared once from calibration-training data only, outside
# every row optimizer, then injected byte-identically into all FK-fixed rows.
FK_ALIGNMENT_SHARED_ROWS = frozenset({"B1", "A3", "B2"})
FK_ALIGNMENT_REQUIRED_PROVENANCE = (
    "artifact_schema",
    "training_set_ids",
    "T_gripper_cam",
    "T_fk_cube_center_to_tag_object",
    "estimation_method",
    "source_observation_ids",
    "source_observation_sha256",
    "raw_fk_source_event_by_set",
    "board_information_used",
    "heldout_information_used",
    "artifact_sha256",
)


def validate_fk_alignment_artifact(artifact: Mapping) -> None:
    missing = [key for key in FK_ALIGNMENT_REQUIRED_PROVENANCE if key not in artifact]
    if missing:
        raise ValueError(f"FK alignment artifact provenance missing {missing}")
    if artifact["artifact_schema"] != "board_free_fk_cube_alignment_v1":
        raise ValueError("FK alignment artifact is not the canonical board-free schema")
    if artifact["board_information_used"] is not False:
        raise ValueError("canonical FK alignment artifact must not use board information")
    if artifact["heldout_information_used"] is not False:
        raise ValueError("canonical FK alignment artifact must not use held-out information")
    if set(artifact.get("canonical_for_rows", ())) != set(FK_ALIGNMENT_SHARED_ROWS):
        raise ValueError("canonical FK alignment artifact consumers must be B1/A3/B2")
    unhashed = dict(artifact)
    expected = str(unhashed.pop("artifact_sha256"))
    encoded = json.dumps(
        unhashed, sort_keys=True, separators=(",", ":")).encode("utf-8")
    actual = hashlib.sha256(encoded).hexdigest()
    if actual != expected:
        raise ValueError("canonical FK alignment artifact SHA-256 mismatch")

# Evaluation policy: a whole-position hold-out cannot score reprojection for an
# estimated per-position cube pose without test-time fitting.  Primary corner
# prediction therefore uses event-level, set-stratified hold-out with every
# estimated pose supported by training events.  Full-position evaluation is a
# separate external-GT or explicitly labelled FK-proxy protocol.
PRIMARY_METRIC = "heldout_event_reprojection_rmse_px"
PRIMARY_SPLIT = "event_grouped_and_set_stratified"
TRAIN_REPROJECTION_ROLE = "optimization_diagnostic_only"
POSITION_HOLDOUT_ROLE = "external_GT_or_explicit_FK_proxy_only"
TASK_POSE_PROXY_LABEL = "e_task_pose^{FK-proxy}"

# Different target sets cannot be ranked on a pooled residual because they do
# not contain the same measurements.  Each declared contrast therefore names
# the only common component that may support that interpretation.
EVALUATION_COMPARISON_CONTRACT = {
    "A0_to_A1": {
        "rows": ("A0", "A1"),
        "components": ("heldout_reprojection.board", "N_reg"),
        "causal_interpretation": "cube_observability_gain_at_seq_and_estimated_pose",
    },
    "A1_to_A2": {
        "rows": ("A1", "A2"),
        "components": ("heldout_reprojection.overall",),
        "causal_interpretation": "unified_feedback_with_estimated_cube_pose",
    },
    "B1_to_A3": {
        "rows": ("B1", "A3"),
        "components": ("heldout_reprojection.overall",),
        "causal_interpretation": "unified_feedback_with_identical_FK_fixed_cube",
    },
    "A2_to_A3": {
        "rows": ("A2", "A3"),
        "components": ("heldout_reprojection.overall",),
        "causal_interpretation": "FK_fixed_cube_pose_effect",
    },
    "B2_to_A3": {
        "rows": ("B2", "A3"),
        "components": ("heldout_reprojection.cube", "e_e2e", "e_cross"),
        "causal_interpretation": "board_addition_given_identical_FK_fixed_cube",
    },
    "B3_to_A3": {
        "rows": ("B3", "A3"),
        "components": ("heldout_reprojection.board",),
        "causal_interpretation": None,
        "interpretation": "board_only_vs_full_system_reference_not_marker_only_causal",
    },
}

NOISE_FREE_SANITY_TOLERANCES = {
    "reprojection_rmse_px": 1e-5,
    "seq_vs_unified_translation_mm": 1e-3,
    "seq_vs_unified_rotation_deg": 1e-5,
}


def _is_soft_anchor(value: str) -> bool:
    return _SOFT_ANCHOR_RE.fullmatch(value) is not None


def validate_condition(condition: AblationCondition) -> None:
    """Reject physically impossible or semantically inconsistent conditions."""
    if condition.target_set not in TARGET_SETS:
        raise ValueError(f"{condition.row}: unknown target set {condition.target_set!r}")
    if condition.unified not in UNIFIED_MODES:
        raise ValueError(f"{condition.row}: unified must be 'seq' or 'U'")

    has_cube = condition.target_set in {"cube", "cube+board"}
    has_board = condition.target_set in {"board", "cube+board"}
    cube_allowed = {POSE_SOURCE_FK_FIXED, POSE_SOURCE_ESTIMATED, POSE_SOURCE_ABSENT}
    if condition.fk_to_cube not in cube_allowed and not _is_soft_anchor(condition.fk_to_cube):
        raise ValueError(f"{condition.row}: invalid FK→cube value {condition.fk_to_cube!r}")
    if has_cube == (condition.fk_to_cube == POSE_SOURCE_ABSENT):
        raise ValueError(f"{condition.row}: FK→cube does not match target presence")
    if _is_soft_anchor(condition.fk_to_cube) and not condition.supplementary:
        raise ValueError(f"{condition.row}: soft-anchor is supplementary-only")

    # A fixed workspace board has no robot-FK pose source.  Keep this explicit:
    # accepting FK-fixed here would silently recreate the old, invalid FK column.
    if condition.fk_to_board not in {POSE_SOURCE_ESTIMATED, POSE_SOURCE_ABSENT}:
        raise ValueError(
            f"{condition.row}: FK→board={condition.fk_to_board!r} is physically invalid; "
            "a fixed external board must be 'estimated' or '—'"
        )
    if has_board == (condition.fk_to_board == POSE_SOURCE_ABSENT):
        raise ValueError(f"{condition.row}: FK→board does not match target presence")


def validate_schema(conditions: Iterable[AblationCondition]) -> None:
    conditions = tuple(conditions)
    rows = [condition.row for condition in conditions]
    if len(rows) != len(set(rows)):
        raise ValueError("ablation row identifiers must be unique")
    for condition in conditions:
        validate_condition(condition)


def validate_main_runner_contract() -> None:
    """Validate the factorial rows and their staged/joint variable partitions."""
    by_row = {condition.row: condition for condition in MAIN_ABLATION_CONDITIONS}
    if set(by_row) != {"A0", "A1", "A2", "A3", "B1", "B2", "B3"}:
        raise ValueError("main ablation must contain exactly A0/A1/A2/A3/B1/B2/B3")
    if set(SEQUENTIAL_STAGE_SPECS) != {row for row, c in by_row.items() if c.unified == "seq"}:
        raise ValueError("sequential stage specs do not match seq rows")
    if set(UNIFIED_FREE_VARIABLES) != {row for row, c in by_row.items() if c.unified == "U"}:
        raise ValueError("unified variable specs do not match U rows")
    if by_row["B2"].unified != "U" or by_row["B3"].unified != "U":
        raise ValueError("B2 and B3 must be U for target-set comparisons")
    actual_fk_fixed = {row for row, c in by_row.items() if c.fk_to_cube == "FK-fixed"}
    if actual_fk_fixed != set(FK_ALIGNMENT_SHARED_ROWS):
        raise ValueError("all and only FK-fixed rows must share one alignment artifact")
    for name, comparison in EVALUATION_COMPARISON_CONTRACT.items():
        if not set(comparison["rows"]).issubset(by_row):
            raise ValueError(f"{name}: comparison references an unknown row")
    if EVALUATION_COMPARISON_CONTRACT["B2_to_A3"]["components"][0] != \
            "heldout_reprojection.cube":
        raise ValueError("B2/A3 must be compared on the common cube component")
    if EVALUATION_COMPARISON_CONTRACT["B3_to_A3"]["causal_interpretation"] is not None:
        raise ValueError("B3/A3 is a whole-system reference, not a marker-only causal contrast")
    for row, spec in SEQUENTIAL_STAGE_SPECS.items():
        if spec.stage2_free != ("T_base_Ci",):
            raise ValueError(f"{row}: seq stage 2 may optimize only T_base_Ci")
        leaked = set(spec.stage1_free) & set(spec.stage2_free)
        if leaked:
            raise ValueError(f"{row}: variables leaked across the seq freeze boundary: {leaked}")


validate_schema(MAIN_ABLATION_CONDITIONS)
validate_schema(SUPPLEMENTARY_CONDITIONS)
validate_main_runner_contract()
