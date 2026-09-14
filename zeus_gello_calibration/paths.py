"""Zeus 촬영 데이터가 있는 위치.

Zeus/GELLO 리그로 찍은 데이터는 이 패키지 안의 ``zeus_gello_calibration/data/``
에 순서대로 쌓는다.  폴더 이름은 촬영 데이터 공통 규칙대로 항상 ``<이름>_<MMDD>`` 이며,
그 규칙의 원본은 :mod:`capture_pipeline.paths` 한 곳이다.
"""

from __future__ import annotations

from pathlib import Path

from capture_pipeline.paths import require_data_path_inside

ZEUS_CALIBRATION_ROOT = Path(__file__).resolve().parent
#: 이 리그의 촬영 데이터 루트.
ZEUS_DATA_ROOT = ZEUS_CALIBRATION_ROOT / "data"

SESSION1_DIR = ZEUS_DATA_ROOT / "session1_handheld_fixed_cam_0909"
SESSION2_DIR = ZEUS_DATA_ROOT / "session2_floor_board_dual_cam_0909"
SESSION3_DIR = ZEUS_DATA_ROOT / "session3_wrist_motion_gripper_cam_0909"


def require_zeus_data_path(path: str | Path, *, label: str = "capture path") -> Path:
    """Return an absolute path only when it is inside ``ZEUS_DATA_ROOT``."""
    return require_data_path_inside(path, ZEUS_DATA_ROOT, label=label)
