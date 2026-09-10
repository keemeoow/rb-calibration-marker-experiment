"""Canonical filesystem paths for Zeus calibration capture data."""

from __future__ import annotations

from pathlib import Path


ZEUS_CALIBRATION_ROOT = Path(__file__).resolve().parent
ZEUS_DATA_ROOT = ZEUS_CALIBRATION_ROOT / "data"

SESSION1_DIR = ZEUS_DATA_ROOT / "session1_handheld_fixed_cam"
SESSION2_DIR = ZEUS_DATA_ROOT / "session2_floor_board_dual_cam"
SESSION3_DIR = ZEUS_DATA_ROOT / "session3_wrist_motion_gripper_cam"


def require_zeus_data_path(path: str | Path, *, label: str = "capture path") -> Path:
    """Return an absolute path only when it is inside ``ZEUS_DATA_ROOT``."""
    candidate = Path(path).expanduser().resolve()
    root = ZEUS_DATA_ROOT.resolve()
    if candidate != root and root not in candidate.parents:
        raise ValueError(
            f"{label} must be inside {root}; received {candidate}"
        )
    return candidate
