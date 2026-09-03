from copy import deepcopy

import pytest

from calibration_pipeline.opencv_relative_baseline import validate_payload
from calibration_pipeline.path_evaluation import (
    E_CROSS_CONTRACT,
    FIXED_TO_FIXED_CROSS_TARGET_CONTRACT,
)
from calibration_pipeline.schema import RELATIVE_POSE_REPORTING_CONTRACT


def _independent_payload():
    contract = RELATIVE_POSE_REPORTING_CONTRACT[
        "independent_reference_baseline"]
    return {
        "protocol": {
            "relative_pose_reporting": RELATIVE_POSE_REPORTING_CONTRACT,
            "independent_reference_contract": contract,
            "uses_fitted_main_method_camera_poses": False,
            "uses_joint_optimizer": False,
            "uses_robot_fk": False,
            "uses_handeye": False,
            "uses_shared_target_pose": False,
        }
    }


def test_method_specific_relative_pose_is_supplementary():
    policy = RELATIVE_POSE_REPORTING_CONTRACT[
        "method_specific_heldout_consistency"]
    assert policy["reporting_tier"] == "supplementary"
    assert policy["may_rank_methods_before_external_gt"] is False
    assert FIXED_TO_FIXED_CROSS_TARGET_CONTRACT[
        "reporting_tier"] == "supplementary"
    assert E_CROSS_CONTRACT["may_rank_methods_before_external_gt"] is False


def test_independent_baseline_rejects_main_method_dependency():
    payload = _independent_payload()
    validate_payload(payload)

    invalid = deepcopy(payload)
    invalid["protocol"]["uses_robot_fk"] = True
    with pytest.raises(ValueError, match="uses_robot_fk"):
        validate_payload(invalid)
