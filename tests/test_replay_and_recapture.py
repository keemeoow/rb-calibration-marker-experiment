from __future__ import annotations

import argparse
import json

import pytest

from zeus_gello_calibration.replay_and_recapture import (
    directory_has_files,
    load_saved_joints,
    require_subdir_name,
)


def _write_robot_json(root, index: str, joints) -> None:
    capture_dir = root / index
    capture_dir.mkdir(parents=True)
    (capture_dir / "robot.json").write_text(json.dumps({"joints": joints}))


def test_load_saved_joints_uses_numeric_capture_order(tmp_path) -> None:
    capture_root = tmp_path / "capture"
    _write_robot_json(capture_root, "010", [10, 11, 12, 13, 14, 15])
    _write_robot_json(capture_root, "002", [2, 3, 4, 5, 6, 7])
    (capture_root / "notes").mkdir()

    items = load_saved_joints(capture_root)

    assert [item["index"] for item in items] == [2, 10]


def test_load_saved_joints_rejects_invalid_six_axis_value(tmp_path) -> None:
    capture_root = tmp_path / "capture"
    _write_robot_json(capture_root, "000", [1, 2, 3])

    with pytest.raises(ValueError, match="숫자 6개"):
        load_saved_joints(capture_root)


@pytest.mark.parametrize("value", ["", ".", "..", "../outside", "nested/output"])
def test_replay_output_must_be_one_subdirectory(value) -> None:
    with pytest.raises(argparse.ArgumentTypeError):
        require_subdir_name(value)

    assert require_subdir_name("capture_replayed_auto") == "capture_replayed_auto"


def test_directory_has_files_distinguishes_empty_output(tmp_path) -> None:
    output = tmp_path / "capture_replayed_auto"
    output.mkdir()
    assert not directory_has_files(output)

    (output / "000").mkdir()
    assert directory_has_files(output)
