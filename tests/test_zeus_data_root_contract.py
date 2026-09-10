from __future__ import annotations

from pathlib import Path

import pytest

from zeus_gello_calibration.paths import (
    ZEUS_DATA_ROOT,
    require_zeus_data_path,
)


ROOT = Path(__file__).resolve().parents[1]


def test_canonical_zeus_data_root_is_inside_the_repository() -> None:
    assert ZEUS_DATA_ROOT == ROOT / "zeus_gello_calibration" / "data"


def test_zeus_data_root_accepts_only_itself_and_descendants() -> None:
    assert require_zeus_data_path(ZEUS_DATA_ROOT) == ZEUS_DATA_ROOT
    capture = ZEUS_DATA_ROOT / "session01" / "calib_train"
    assert require_zeus_data_path(capture) == capture

    with pytest.raises(ValueError, match="must be inside"):
        require_zeus_data_path(ROOT / "data" / "session05")
    with pytest.raises(ValueError, match="must be inside"):
        require_zeus_data_path(ROOT / "zeus_gello_calibration" / "other_data")
