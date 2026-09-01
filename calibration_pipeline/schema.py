"""Canonical method schema and comparison contract for Table 1.

Robot FK ``T_base_gripper`` is the shared kinematic backbone in every condition.
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
POSE_SOURCE_FK_FACTOR = "corrected-FK-factor"
POSE_SOURCE_ABSENT = "—"
DEFAULT_SPLIT_SEED = 20260731
_SOFT_ANCHOR_RE = re.compile(r"soft-anchor \(λ=(?:0|[1-9][0-9]*)(?:\.[0-9]+)?\)")

CORRECTED_FK_FACTOR_CONTRACT = {
    "aligned_pose": "T_fk_aligned_set = T_fk_raw_set @ Delta_train",
    "delta_estimation": (
        "board_free_training_eye_in_hand_cube_corner_reprojection"),
    "cube_pose_is_optimization_variable": True,
    "factor_residual": (
        "r_set=[Log_SO3(R_cube^T R_fk_aligned), "
        "translation(inv(T_cube)@T_fk_aligned)]"),
    "whitening": "w_set=chol(Sigma_set)^(-1)@r_set",
    "objective": "robust_visual_corner_residuals_plus_robust_whitened_FK_factors",
    "hard_gate_or_pose_replacement": False,
    "external_ground_truth_used": False,
}


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
    AblationCondition(
        "A4", "cube+board", "U", "corrected-FK-factor", "estimated",
        "Ours (corrected-FK factor)",
    ),
    AblationCondition(
        "B1", "cube+board", "seq", "corrected-FK-factor", "estimated",
        "−Unified",
    ),
    AblationCondition("B2", "cube", "U", "corrected-FK-factor", "—", "−board"),
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

# Exact eih/e2h optimization structure implemented by run_condition_once.
# ``seq`` is mathematically camera-separable in stage 2 because target and
# hand-eye variables are frozen; ``U`` stacks both observation families in one
# residual vector and jointly updates every row-declared free variable.
OPTIMIZATION_STRUCTURE_CONTRACT = {
    "seq": {
        "stage_1": {
            "observations": "eye_in_hand_only",
            "objective": "sum_of_robust_raw_corner_reprojection_residuals",
            "solves": "T_gripper_camera_and_declared_target_poses",
        },
        "freeze_boundary": (
            "all_stage_1_estimates_are_constants_during_stage_2"),
        "stage_2": {
            "observations": "fixed_eye_to_hand_only",
            "objective": "sum_of_robust_raw_corner_reprojection_residuals",
            "solves": "each_T_base_fixed_camera_only",
            "camera_blocks_are_separable": True,
        },
        "feedback_from_e2h_to_eih_variables": False,
        "alternating_pass": False,
    },
    "U": {
        "observations": "eye_in_hand_union_fixed_eye_to_hand",
        "objective": "one_stacked_robust_raw_corner_reprojection_vector",
        "solves": "T_base_fixed_cameras_T_gripper_camera_and_declared_target_poses",
        "coupling": "shared_target_pose_variables_and_common_handeye",
        "feedback_from_e2h_to_eih_variables": True,
        "e_cross_is_an_objective_term": False,
    },
}


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
        ("T_gripper_cam", "T_base_board", "T_base_cube_by_set"),
        GLOBAL_FIXED_INPUTS + ("soft_FK_factor_covariance",),
        "e2h_only",
        ("T_base_Ci",),
        ("T_gripper_cam", "T_base_board", "T_base_cube_by_set"),
    ),
}

# Joint rows use both camera paths in one objective.  These lists are the only
# permitted optimization variables for the corresponding rows.
UNIFIED_FREE_VARIABLES: Mapping[str, tuple[str, ...]] = {
    "A2": ("T_base_Ci", "T_gripper_cam", "T_base_board", "T_base_cube_by_set"),
    "A3": ("T_base_Ci", "T_gripper_cam", "T_base_board"),
    "A4": ("T_base_Ci", "T_gripper_cam", "T_base_board", "T_base_cube_by_set"),
    "B2": ("T_base_Ci", "T_gripper_cam", "T_base_cube_by_set"),
    "B3": ("T_base_Ci", "T_gripper_cam", "T_base_board"),
}

# This artifact is prepared once from calibration-training data only, outside
# every row optimizer. A3 consumes it as a hard fixed pose; A4/B1/B2 consume
# the same pose as the centre of the preregistered soft FK factor.
FK_FIXED_ROWS = frozenset({"A3"})
FK_ALIGNMENT_SHARED_ROWS = frozenset({"A3", "A4", "B1", "B2"})
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
        raise ValueError("canonical FK alignment artifact consumers must be A3/A4/B1/B2")
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
# ``PRIMARY_METRIC`` ranks only rows with the same marker population.  Across
# board-only/cube-only/combined systems, use the target-specific camera-scope
# metrics until independent external GT is available.
PRE_GT_CROSS_METHOD_PRIMARY = (
    "two_scope_target_specific_cross_view:FK_free_fixed_to_fixed_plus_"
    "FK_dependent_gripper_to_fixed")
SHARED_TARGET_REPROJECTION_ROLE = (
    "secondary_reference_dependent_diagnostic_not_for_cross_method_ranking")

# Table 1 is a component ablation: every row starts from the same train-only
# reference state.  In particular, its marker-removal rows answer what happens
# when a marker residual/variable is removed *after shared initialization*.
# They are not end-to-end board-only/cube-only system comparisons.  The latter
# are implemented by ``Run_calibration_comparison.py marker-system`` with modality-specific
# initializers and one identical evaluation contract.
MARKER_COMPARISON_CONTRACT = {
    "optimization_level": {
        "runner": "Run_calibration_comparison.py table1",
        "initialization": "one_shared_train_only_all_marker_reference_state",
        "interpretation": "marker_residual_and_variable_contribution",
        "may_claim_end_to_end_marker_system_performance": False,
    },
    "end_to_end_system": {
        "runner": "Run_calibration_comparison.py marker-system",
        "initialization": "train_only_marker_modality_specific",
        "optimization": "same_marker_modality_unified_visual_objective",
        "fixed_across_systems": (
            "train_test_split", "raw_detections", "camera_intrinsics_K",
            "distortion_D", "solver_options", "initialization_seeds",
            "heldout_fixed_to_fixed_board_cube_mask",
            "heldout_gripper_to_fixed_board_cube_mask",
        ),
        "interpretation": (
            "marker_system_relative_performance_under_target_specific_fixed_"
            "subsystem_and_gripper_to_fixed_full_chain_evaluation"),
        "shared_target_pose_reprojection_role": (
            "secondary_reference_dependent_diagnostic_not_for_ranking"),
        "may_claim_external_absolute_accuracy": False,
    },
}

# Different target sets cannot be ranked on a pooled residual because they do
# not contain the same measurements.  Each declared contrast therefore names
# the only shared component that may support that interpretation.
EVALUATION_COMPARISON_CONTRACT = {
    "A0_to_A1": {
        "rows": ("A0", "A1"),
        "components": ("heldout_reprojection.board", "N_reg"),
        "causal_interpretation": (
            "optimization_level_cube_residual_contribution_at_shared_initialization"
        ),
    },
    "A1_to_A2": {
        "rows": ("A1", "A2"),
        "components": ("heldout_reprojection.overall",),
        "causal_interpretation": "unified_feedback_with_estimated_cube_pose",
    },
    "B1_to_A4": {
        "rows": ("B1", "A4"),
        "components": ("heldout_reprojection.overall",),
        "causal_interpretation": "unified_feedback_with_identical_soft_FK_factor",
    },
    "A2_to_A3": {
        "rows": ("A2", "A3"),
        "components": ("heldout_reprojection.overall",),
        "causal_interpretation": "FK_fixed_cube_pose_effect",
    },
    "B2_to_A4": {
        "rows": ("B2", "A4"),
        "components": ("heldout_reprojection.cube", "e_e2e", "e_cross"),
        "causal_interpretation": (
            "optimization_level_board_residual_contribution_given_identical_soft_FK_factor"
        ),
    },
    "B3_to_A2": {
        "rows": ("B3", "A2"),
        "components": ("heldout_reprojection.board", "e_e2e", "e_cross"),
        "causal_interpretation": (
            "optimization_level_cube_residual_contribution_at_shared_initialization"
        ),
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
    cube_allowed = {
        POSE_SOURCE_FK_FIXED,
        POSE_SOURCE_FK_FACTOR,
        POSE_SOURCE_ESTIMATED,
        POSE_SOURCE_ABSENT,
    }
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
    if set(by_row) != {"A0", "A1", "A2", "A3", "A4", "B1", "B2", "B3"}:
        raise ValueError(
            "main ablation must contain exactly A0/A1/A2/A3/A4/B1/B2/B3")
    if set(SEQUENTIAL_STAGE_SPECS) != {row for row, c in by_row.items() if c.unified == "seq"}:
        raise ValueError("sequential stage specs do not match seq rows")
    if set(UNIFIED_FREE_VARIABLES) != {row for row, c in by_row.items() if c.unified == "U"}:
        raise ValueError("unified variable specs do not match U rows")
    if by_row["B2"].unified != "U" or by_row["B3"].unified != "U":
        raise ValueError("B2 and B3 must be U for target-set comparisons")
    actual_fk_fixed = {row for row, c in by_row.items() if c.fk_to_cube == "FK-fixed"}
    if actual_fk_fixed != set(FK_FIXED_ROWS):
        raise ValueError("A3 must be the only hard FK-fixed row")
    canonical_methods = set(by_row)
    for name, comparison in EVALUATION_COMPARISON_CONTRACT.items():
        if not set(comparison["rows"]).issubset(canonical_methods):
            raise ValueError(f"{name}: comparison references an unknown row")
    if EVALUATION_COMPARISON_CONTRACT["B2_to_A4"]["components"][0] != \
            "heldout_reprojection.cube":
        raise ValueError("B2/A4 must be compared on the shared cube component")
    if EVALUATION_COMPARISON_CONTRACT["B3_to_A3"]["causal_interpretation"] is not None:
        raise ValueError("B3/A3 is a whole-system reference, not a marker-only causal contrast")
    if MARKER_COMPARISON_CONTRACT["optimization_level"][
            "may_claim_end_to_end_marker_system_performance"]:
        raise ValueError("shared-initialization rows cannot claim marker-system performance")
    if MARKER_COMPARISON_CONTRACT["end_to_end_system"][
            "may_claim_external_absolute_accuracy"]:
        raise ValueError("internal end-to-end evaluation cannot claim absolute accuracy")
    for row, spec in SEQUENTIAL_STAGE_SPECS.items():
        if spec.stage2_free != ("T_base_Ci",):
            raise ValueError(f"{row}: seq stage 2 may optimize only T_base_Ci")
        leaked = set(spec.stage1_free) & set(spec.stage2_free)
        if leaked:
            raise ValueError(f"{row}: variables leaked across the seq freeze boundary: {leaked}")


validate_schema(MAIN_ABLATION_CONDITIONS)
validate_schema(SUPPLEMENTARY_CONDITIONS)
validate_main_runner_contract()
