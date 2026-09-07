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
POSE_SOURCE_FK_FIXED = "raw-FK-fixed"
POSE_SOURCE_ALIGNED_FK_FIXED = "vision-aligned-FK-fixed"
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
    # NOT "Ours (full)": A3 is the most *constrained* row, not the one that
    # uses the most.  It removes 6 DoF per set from the optimizer.
    AblationCondition("A3", "cube+board", "U", POSE_SOURCE_FK_FIXED, "estimated",
                      "raw-FK hard fixed"),
    AblationCondition(
        "A4", "cube+board", "U", "corrected-FK-factor", "estimated",
        "corrected-FK soft factor",
    ),
    AblationCondition(
        "A5", "cube+board", "U", POSE_SOURCE_ALIGNED_FK_FIXED, "estimated",
        "vision-aligned FK hard fixed",
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

# Neither optimization structure ever estimates or constrains a camera-to-camera
# transform.  Fixed cameras are coupled only because they reproject onto the same
# shared target-pose variables, so ``T_Ci_Cj`` is a quantity derived from a
# converged solution - never a residual, an observation, or an objective term.
# The SE(3) averaging in :mod:`calibration_pipeline.se3` is initialization only.
INTER_CAMERA_COUPLING_CONTRACT = {
    "camera_to_camera_residual": False,
    "camera_to_camera_observation": False,
    "relative_pose_chaining_or_averaging_inside_the_estimator": False,
    "camera_coupling_mechanism": "shared_target_pose_variables_only",
    "T_Ci_Cj": "derived_as_inv(T_base_Ci)@T_base_Cj_after_the_solve",
    "se3_averaging_scope": "initialization_only",
    "e_cross_is_an_objective_term": False,
    "forbidden_description": (
        "per-camera cube poses are compared across events to build a "
        "representative camera-to-camera transform that is then integrated"),
}

# Relative-pose quantities are reported in two deliberately different roles.
# A is computed from each fitted method and is therefore useful only as a
# held-out self-consistency diagnostic.  B is fitted directly from measurement-
# only OpenCV PnP candidates and remains independent of every main-method pose.
RELATIVE_POSE_REPORTING_CONTRACT = {
    "method_specific_heldout_consistency": {
        "label": "A",
        "implementation": "fixed_to_fixed_board_cube_and_legacy_e_cross",
        "reporting_tier": "supplementary",
        "uses_fitted_main_method_camera_poses": True,
        "uses_robot_fk": False,
        "uses_external_ground_truth": False,
        "may_rank_methods_before_external_gt": False,
        "interpretation": (
            "heldout consistency of each fitted method; not an independent "
            "relative-pose reference and not absolute physical accuracy"),
    },
    "independent_reference_baseline": {
        "label": "B",
        "runner": "calibration_pipeline.opencv_relative_baseline:main",
        "implementation": (
            "measurement_only_OpenCV_PnP_direct_relative_transforms_and_"
            "preregistered_robust_SE3_average"),
        "reporting_tier": "independent_reference",
        "uses_fitted_main_method_camera_poses": False,
        "uses_joint_optimizer": False,
        "uses_robot_fk": False,
        "uses_handeye": False,
        "uses_shared_target_pose": False,
        "uses_external_ground_truth": False,
        "absolute_accuracy_metric": False,
        "interpretation": (
            "FK-free independent relative-pose reference baseline; not SOTA "
            "and not external ground truth"),
    },
}

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
        "inter_camera_coupling": INTER_CAMERA_COUPLING_CONTRACT,
    },
    "U": {
        "observations": "eye_in_hand_union_fixed_eye_to_hand",
        "objective": "one_stacked_robust_raw_corner_reprojection_vector",
        "solves": "T_base_fixed_cameras_T_gripper_camera_and_declared_target_poses",
        "coupling": "shared_target_pose_variables_and_common_handeye",
        "feedback_from_e2h_to_eih_variables": True,
        "e_cross_is_an_objective_term": False,
        "inter_camera_coupling": INTER_CAMERA_COUPLING_CONTRACT,
    },
}

# How many additive terms the minimized objective actually contains.  There is
# no scalar term weight anywhere in the estimator: the visual term is scaled
# only by its pixel ``f_scale`` and the FK term only by ``Sigma^(-1/2)``.  A
# "w1*reprojection + w2*pose_error + w3*FK_constraint" description is therefore
# wrong for every row - no row has three terms, and most rows have exactly one.
# "pose error" and "FK constraint" are two names for the same single term.
OBJECTIVE_TERM_CONTRACT = {
    "visual_only": {
        "rows": ("A0", "A1", "A2", "A3", "A5", "B3"),
        "n_objective_terms": 1,
        "terms": ("robust_corner_reprojection",),
        "pose_error_term": False,
        "fk_constraint_term": False,
    },
    "visual_plus_fk_factor": {
        "rows": ("A4", "B1", "B2"),
        "n_objective_terms": 2,
        "terms": ("robust_corner_reprojection", "whitened_robust_FK_factor"),
        "pose_error_term": "same_term_as_the_fk_factor_not_a_third_term",
        "fk_constraint_term": True,
    },
    "max_objective_terms_in_any_row": 2,
    "scalar_term_weights_used": False,
    "relative_weighting": (
        "visual f_scale in pixels and FK Sigma^(-1/2) only; no w1/w2/w3"),
    "forbidden_description": (
        "weighted sum of a reprojection term, a separate pose-error term, and "
        "a separate FK-constraint term"),
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
    "A5": ("T_base_Ci", "T_gripper_cam", "T_base_board"),
    "B2": ("T_base_Ci", "T_gripper_cam", "T_base_cube_by_set"),
    "B3": ("T_base_Ci", "T_gripper_cam", "T_base_board"),
}

# A3 uses no image-fitted FK-to-object alignment. Controller tool4 and the cube
# model both place their origin at the cube center, but their axes differ by an
# exact 180-degree rotation about Y in the upright grasp. This preregistered
# mechanical frame map is a coordinate conversion, not an estimated correction.
RAW_FK_CUBE_CENTER_TO_OBJECT = (
    (-1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, -1.0, 0.0),
    (0.0, 0.0, 0.0, 1.0),
)

FK_FIXED_CONTRACT = {
    "fixed_pose": (
        "T_base_cube[s] = T_base_fk_raw[s] @ "
        "T_cube_center_tag_object_mechanical"),
    "raw_fk_source": "taught set_cube_center robot TCP pose, one per set",
    "mechanical_frame_map": RAW_FK_CUBE_CENTER_TO_OBJECT,
    "mechanical_frame_map_source": (
        "controller tool4 frame and configured cube-object frame definitions"),
    "image_fitted_fk_to_object_transform_used": False,
    "per_set_variation_source": "robot_FK_only",
    "vision_degrees_of_freedom_in_the_fixed_cube_target": 0,
    "degrees_of_freedom_removed_from_the_row_optimizer": "6_per_set",
    "external_ground_truth_used": False,
    "forbidden_description": (
        "A3 uses the vision-aligned FK artifact or Delta_train"),
    "accurate_description": (
        "the per-set cube target is fixed from raw robot FK after a "
        "preregistered mechanical frame-coordinate conversion"),
}

VISION_ALIGNED_FK_FIXED_CONTRACT = {
    "fixed_pose": "T_base_cube[s] = T_base_fk_raw[s] @ Delta_train",
    "delta_estimation": (
        "board_free_training_eye_in_hand_cube_corner_reprojection"),
    "image_fitted_fk_to_object_transform_used": True,
    "training_information_used": True,
    "heldout_information_used": False,
    "cube_pose_is_optimization_variable": False,
    "degrees_of_freedom_removed_from_the_row_optimizer": "6_per_set",
    "external_ground_truth_used": False,
    "reporting_role": "final_candidate_if_preregistered_before_external_GT",
    "accurate_description": (
        "train-only vision-aligned FK cube poses are hard-fixed during A5"),
    "forbidden_description": (
        "A5 uses independent physical correction labels or external GT"),
}

# This image-aligned artifact is prepared once from calibration-training data
# only, outside every row optimizer. It is the centre of the A4/B1/B2 soft FK
# factor and the hard-fixed target for the A5 final-candidate row. A3
# deliberately does not consume it.
RAW_FK_FIXED_ROWS = frozenset({"A3"})
ALIGNED_FK_FIXED_ROWS = frozenset({"A5"})
FK_FIXED_ROWS = RAW_FK_FIXED_ROWS | ALIGNED_FK_FIXED_ROWS
FK_ALIGNMENT_SHARED_ROWS = frozenset({"A4", "A5", "B1", "B2"})
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


def validate_objective_contracts() -> None:
    """Fail closed when a declared contract drifts from the row definitions.

    These are the three descriptions that have been misreported about this
    pipeline, so each one is checked against the structures that actually
    drive the optimizer rather than kept as prose.
    """
    rows = {condition.row for condition in MAIN_ABLATION_CONDITIONS}
    declared = (tuple(OBJECTIVE_TERM_CONTRACT["visual_only"]["rows"])
                + tuple(OBJECTIVE_TERM_CONTRACT["visual_plus_fk_factor"]["rows"]))
    if sorted(declared) != sorted(rows):
        raise ValueError(
            "OBJECTIVE_TERM_CONTRACT must classify every main row exactly once: "
            f"declared={sorted(declared)}, rows={sorted(rows)}")
    if OBJECTIVE_TERM_CONTRACT["max_objective_terms_in_any_row"] != max(
            OBJECTIVE_TERM_CONTRACT[group]["n_objective_terms"]
            for group in ("visual_only", "visual_plus_fk_factor")):
        raise ValueError("declared maximum objective-term count is inconsistent")
    for row in FK_FIXED_ROWS:
        if "T_base_cube_by_set" in UNIFIED_FREE_VARIABLES.get(row, ()):
            raise ValueError(
                f"{row} is declared FK-fixed but still frees T_base_cube_by_set")
    for condition in MAIN_ABLATION_CONDITIONS:
        if condition.label == "Ours (full)":
            raise ValueError(
                "'Ours (full)' overstates a row that removes degrees of freedom; "
                "name the treatment instead")
    method_specific = RELATIVE_POSE_REPORTING_CONTRACT[
        "method_specific_heldout_consistency"]
    if (method_specific["reporting_tier"] != "supplementary"
            or method_specific["may_rank_methods_before_external_gt"] is not False):
        raise ValueError(
            "method-specific relative-pose consistency must remain supplementary")
    independent = RELATIVE_POSE_REPORTING_CONTRACT[
        "independent_reference_baseline"]
    forbidden_dependencies = (
        "uses_fitted_main_method_camera_poses", "uses_joint_optimizer",
        "uses_robot_fk", "uses_handeye", "uses_shared_target_pose",
    )
    if (independent["reporting_tier"] != "independent_reference"
            or any(independent[key] is not False for key in forbidden_dependencies)):
        raise ValueError(
            "independent relative-pose baseline acquired a main-method dependency")


validate_objective_contracts()


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
        raise ValueError(
            "canonical FK alignment artifact consumers must be A4/A5/B1/B2")
    unhashed = dict(artifact)
    expected = str(unhashed.pop("artifact_sha256"))
    encoded = json.dumps(
        unhashed, sort_keys=True, separators=(",", ":")).encode("utf-8")
    actual = hashlib.sha256(encoded).hexdigest()
    if actual != expected:
        raise ValueError("canonical FK alignment artifact SHA-256 mismatch")

# Evaluation policy: final ranking is cube-only.  External cube GT is the
# primary physical metric; internal cube reprojection and camera consistency are
# retained only as supporting diagnostics.
FINAL_EVALUATION_TARGET = "cube"
PRIMARY_METRIC = "external_cube_TRE_rotation_P95_failure"
PRIMARY_INTERNAL_METRIC = "heldout_cube_reprojection_rmse_px"
PRIMARY_SPLIT = "event_grouped_and_set_stratified"
TRAIN_REPROJECTION_ROLE = "optimization_diagnostic_only"
POSITION_HOLDOUT_ROLE = "external_GT_or_explicit_FK_proxy_only"
TASK_POSE_PROXY_LABEL = "e_task_pose^{FK-proxy}"
# ``PRIMARY_METRIC`` ranks only rows with the same marker population.  The
# method-specific fixed-to-fixed/e_cross values are supplementary and may not
# become a cross-method primary merely because external GT is unavailable.
PRE_GT_CROSS_METHOD_PRIMARY = "none_without_independent_external_ground_truth"
METHOD_SPECIFIC_HELDOUT_CONSISTENCY_ROLE = (
    "supplementary_not_for_cross_method_ranking")
SHARED_TARGET_REPROJECTION_ROLE = (
    "secondary_reference_dependent_diagnostic_not_for_cross_method_ranking")

# Table 1 is a component ablation: every row starts from the same train-only
# reference state.  In particular, its marker-removal rows answer what happens
# when a marker residual/variable is removed *after shared initialization*.
# They are not end-to-end board-only/cube-only system comparisons.  The latter
# are implemented by ``calibration_pipeline.marker_system`` with modality-specific
# initializers and one identical evaluation contract.
MARKER_COMPARISON_CONTRACT = {
    "optimization_level": {
        "runner": "calibration_pipeline.table1:main",
        "initialization": "one_shared_train_only_all_marker_reference_state",
        "interpretation": "marker_residual_and_variable_contribution",
        "may_claim_end_to_end_marker_system_performance": False,
    },
    "end_to_end_system": {
        "runner": "calibration_pipeline.marker_system:main",
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
        "fixed_to_fixed_reporting_role": (
            "supplementary_method_specific_heldout_consistency"),
        "shared_target_pose_reprojection_role": (
            "secondary_reference_dependent_diagnostic_not_for_ranking"),
        "may_claim_external_absolute_accuracy": False,
    },
}

# Different target sets cannot be ranked on a pooled board/cube residual because
# they do not contain the same measurements.  Final comparison therefore uses a
# single shared evaluation target: cube held-out / External cube GT.
EVALUATION_COMPARISON_CONTRACT = {
    "A0_to_B3": {
        "rows": ("A0", "B3"),
        "components": ("external_gt.cube", "heldout_reprojection.cube"),
        "supplementary_components": (
            "cross_view_pixel_transfer.cube",
            "cam_common_obj_cam.cube",
        ),
        "evidence_tier": "final_protocol",
        "causal_interpretation": (
            "board_on_gripper_only_sequential_vs_unified_feedback_with_cube_evaluation"
        ),
    },
    "A0_to_A1": {
        "rows": ("A0", "A1"),
        "components": ("external_gt.cube", "heldout_reprojection.cube"),
        "supplementary_components": (
            "cross_view_pixel_transfer.cube",
            "cam_common_obj_cam.cube",
        ),
        "causal_interpretation": (
            "cube_training_residual_contribution_against_board_on_gripper_baseline"
        ),
    },
    "A1_to_A2": {
        "rows": ("A1", "A2"),
        "components": ("external_gt.cube", "heldout_reprojection.cube"),
        "supplementary_components": (
            "cross_view_pixel_transfer.cube",
            "cam_common_obj_cam.cube",
        ),
        "causal_interpretation": "unified_feedback_with_estimated_cube_pose",
    },
    "B1_to_A4": {
        "rows": ("B1", "A4"),
        "components": ("external_gt.cube", "heldout_reprojection.cube"),
        "supplementary_components": (
            "cross_view_pixel_transfer.cube",
            "cam_common_obj_cam.cube",
        ),
        "causal_interpretation": "unified_feedback_with_identical_soft_FK_factor",
    },
    "A2_to_A4": {
        "rows": ("A2", "A4"),
        "components": ("external_gt.cube", "heldout_reprojection.cube"),
        "supplementary_components": (
            "cross_view_pixel_transfer.cube",
            "cam_common_obj_cam.cube",
        ),
        "evidence_tier": "final_protocol",
        "causal_interpretation": (
            "soft_FK_factor_added_to_unified_estimated_cube_pose"),
        "reporting_limit": (
            "A4_uses_simulation_prior_covariance_until_measured_FK_covariance_is_supplied"),
    },
    "A2_to_A3": {
        "rows": ("A2", "A3"),
        "components": ("external_gt.cube", "heldout_reprojection.cube"),
        "supplementary_components": (
            "cross_view_pixel_transfer.cube",
            "cam_common_obj_cam.cube",
        ),
        "causal_interpretation": "FK_fixed_cube_pose_effect",
    },
    "A3_to_A5": {
        "rows": ("A3", "A5"),
        "components": ("external_gt.cube", "heldout_reprojection.cube"),
        "supplementary_components": (
            "cross_view_pixel_transfer.cube",
            "cam_common_obj_cam.cube",
        ),
        "causal_interpretation": (
            "raw_mechanical_FK_fixed_vs_train_vision_aligned_FK_fixed"),
    },
    "A4_to_A5": {
        "rows": ("A4", "A5"),
        "components": ("external_gt.cube", "heldout_reprojection.cube"),
        "supplementary_components": (
            "cross_view_pixel_transfer.cube",
            "cam_common_obj_cam.cube",
        ),
        "causal_interpretation": (
            "soft_FK_factor_vs_hard_fixed_aligned_FK_with_shared_alignment"),
        "reporting_limit": (
            "A5_can_be_final_only_if_frozen_before_external_GT_scoring"),
    },
    "B2_to_A4": {
        "rows": ("B2", "A4"),
        "components": ("external_gt.cube", "heldout_reprojection.cube"),
        "supplementary_components": (
            "cross_view_pixel_transfer.cube",
            "cam_common_obj_cam.cube",
        ),
        "causal_interpretation": (
            "optimization_level_board_residual_contribution_given_identical_soft_FK_factor"
        ),
    },
    "B3_to_A2": {
        "rows": ("B3", "A2"),
        "components": ("external_gt.cube", "heldout_reprojection.cube"),
        "supplementary_components": (
            "cross_view_pixel_transfer.cube",
            "cam_common_obj_cam.cube",
        ),
        "causal_interpretation": (
            "optimization_level_cube_residual_contribution_at_shared_initialization"
        ),
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
        POSE_SOURCE_ALIGNED_FK_FIXED,
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

    # Final A0/B3 use a board mounted on the gripper for pose diversity, but the
    # final table still keeps only the declared A0/B3 rows.  Do not reject board
    # FK metadata if a future board-only final run records it.
    board_allowed = {
        POSE_SOURCE_ESTIMATED,
        POSE_SOURCE_FK_FIXED,
        POSE_SOURCE_ALIGNED_FK_FIXED,
        POSE_SOURCE_FK_FACTOR,
        POSE_SOURCE_ABSENT,
    }
    if condition.fk_to_board not in board_allowed:
        raise ValueError(
            f"{condition.row}: invalid FK→board value {condition.fk_to_board!r}"
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
    if set(by_row) != {"A0", "A1", "A2", "A3", "A4", "A5", "B1", "B2", "B3"}:
        raise ValueError(
            "main ablation must contain exactly A0/A1/A2/A3/A4/A5/B1/B2/B3")
    if set(SEQUENTIAL_STAGE_SPECS) != {row for row, c in by_row.items() if c.unified == "seq"}:
        raise ValueError("sequential stage specs do not match seq rows")
    if set(UNIFIED_FREE_VARIABLES) != {row for row, c in by_row.items() if c.unified == "U"}:
        raise ValueError("unified variable specs do not match U rows")
    if by_row["B2"].unified != "U" or by_row["B3"].unified != "U":
        raise ValueError("B2 and B3 must be U for target-set comparisons")
    actual_raw_fk_fixed = {
        row for row, c in by_row.items()
        if c.fk_to_cube == POSE_SOURCE_FK_FIXED
    }
    if actual_raw_fk_fixed != set(RAW_FK_FIXED_ROWS):
        raise ValueError("A3 must be the only raw hard FK-fixed row")
    actual_aligned_fk_fixed = {
        row for row, c in by_row.items()
        if c.fk_to_cube == POSE_SOURCE_ALIGNED_FK_FIXED
    }
    if actual_aligned_fk_fixed != set(ALIGNED_FK_FIXED_ROWS):
        raise ValueError("A5 must be the only vision-aligned hard FK-fixed row")
    canonical_methods = set(by_row)
    for name, comparison in EVALUATION_COMPARISON_CONTRACT.items():
        if not set(comparison["rows"]).issubset(canonical_methods):
            raise ValueError(f"{name}: comparison references an unknown row")
    final_components = ("external_gt.cube", "heldout_reprojection.cube")
    final_supplementary = {
        "cross_view_pixel_transfer.cube",
        "cam_common_obj_cam.cube",
    }
    for name, comparison in EVALUATION_COMPARISON_CONTRACT.items():
        if comparison["components"] != final_components:
            raise ValueError(f"{name}: final comparison must be cube-only")
        if set(comparison.get("supplementary_components", ())) != final_supplementary:
            raise ValueError(f"{name}: final supplementary metrics drifted")
        comparison = EVALUATION_COMPARISON_CONTRACT[name]
        forbidden = {
            "heldout_reprojection.board",
            "heldout_reprojection.overall",
            "e_e2e",
            "e_cross",
        }
        if forbidden & set(comparison["components"]):
            raise ValueError(f"{name}: old internal metric survived in components")
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
