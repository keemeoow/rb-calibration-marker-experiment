# config.py
"""Project configuration and the single source of truth for the 59mm AprilTag cube.

Units: every field ending with ``_m`` is meters.
Only edit the marker-ID block when the cube is reprinted with different IDs.
"""

from dataclasses import dataclass, field
from typing import Dict, Tuple


# =============================================================================
# USER-EDITABLE MARKER IDS / SIZES - AprilTag 59mm calibration cube
# -----------------------------------------------------------------------------
# Edit only this block when reprinting tags with different IDs.
# Top face has two 25mm tags on +Z, centered at y=-14mm and y=+14mm.
# Four side faces have one 51mm tag each.
# =============================================================================
TOP_MARKER_NEG_Y_ID = 0   # +Z top face, center: (0, -14, +29.5) mm
TOP_MARKER_POS_Y_ID = 1   # +Z top face, center: (0, +14, +29.5) mm
SIDE_MARKER_POS_X_ID = 2  # +X side face, center: (+29.5, 0, -1) mm
SIDE_MARKER_POS_Y_ID = 3  # +Y side face, center: (0, +29.5, -1) mm
SIDE_MARKER_NEG_X_ID = 4  # -X side face, center: (-29.5, 0, -1) mm
SIDE_MARKER_NEG_Y_ID = 5  # -Y side face, center: (0, -29.5, -1) mm

TOP_MARKER_SIZE_M = 0.025
SIDE_MARKER_SIZE_M = 0.051
# =============================================================================


@dataclass
class CubeConfig:
    """Physical definition of the 59 x 59 x 59mm AprilTag cube.

    Object frame:
      - origin: center of the full 59mm bounding cube
      - +Z: upward; top surface z = +29.5mm
      - side marker centers are at z = -1mm because the 57mm lower body spans
        z = -29.5mm .. +27.5mm.
    """

    cube_side_m: float = 0.059
    marker_size_m: float = SIDE_MARKER_SIZE_M  # fallback only
    dictionary_name: str = "DICT_APRILTAG_36h11"

    marker_ids: Tuple[int, ...] = (
        TOP_MARKER_NEG_Y_ID,
        TOP_MARKER_POS_Y_ID,
        SIDE_MARKER_POS_X_ID,
        SIDE_MARKER_POS_Y_ID,
        SIDE_MARKER_NEG_X_ID,
        SIDE_MARKER_NEG_Y_ID,
    )

    id_to_face: Dict[int, str] = field(default_factory=lambda: {
        TOP_MARKER_NEG_Y_ID: "+Z",
        TOP_MARKER_POS_Y_ID: "+Z",
        SIDE_MARKER_POS_X_ID: "+X",
        SIDE_MARKER_POS_Y_ID: "+Y",
        SIDE_MARKER_NEG_X_ID: "-X",
        SIDE_MARKER_NEG_Y_ID: "-Y",
    })

    marker_size_by_id: Dict[int, float] = field(default_factory=lambda: {
        TOP_MARKER_NEG_Y_ID: TOP_MARKER_SIZE_M,
        TOP_MARKER_POS_Y_ID: TOP_MARKER_SIZE_M,
        SIDE_MARKER_POS_X_ID: SIDE_MARKER_SIZE_M,
        SIDE_MARKER_POS_Y_ID: SIDE_MARKER_SIZE_M,
        SIDE_MARKER_NEG_X_ID: SIDE_MARKER_SIZE_M,
        SIDE_MARKER_NEG_Y_ID: SIDE_MARKER_SIZE_M,
    })

    marker_center_m: Dict[int, Tuple[float, float, float]] = field(default_factory=lambda: {
        TOP_MARKER_NEG_Y_ID: (0.0, -0.014, 0.0295),
        TOP_MARKER_POS_Y_ID: (0.0, 0.014, 0.0295),
        SIDE_MARKER_POS_X_ID: (0.0295, 0.0, -0.001),
        SIDE_MARKER_POS_Y_ID: (0.0, 0.0295, -0.001),
        SIDE_MARKER_NEG_X_ID: (-0.0295, 0.0, -0.001),
        SIDE_MARKER_NEG_Y_ID: (0.0, -0.0295, -0.001),
    })

    # Detector corner order correction. Keep identity unless a printed tag is
    # physically rotated/mirrored and you have verified the required order.
    corner_reorder: Dict[int, Tuple[int, int, int, int]] = field(default_factory=lambda: {
        TOP_MARKER_NEG_Y_ID: (0, 1, 2, 3),
        TOP_MARKER_POS_Y_ID: (0, 1, 2, 3),
        SIDE_MARKER_POS_X_ID: (0, 1, 2, 3),
        SIDE_MARKER_POS_Y_ID: (0, 1, 2, 3),
        SIDE_MARKER_NEG_X_ID: (0, 1, 2, 3),
        SIDE_MARKER_NEG_Y_ID: (0, 1, 2, 3),
    })

    # In-plane rotation around each face normal, degrees. Validate physically.
    face_roll_deg: Dict[int, float] = field(default_factory=lambda: {
        TOP_MARKER_NEG_Y_ID: 0.0,
        TOP_MARKER_POS_Y_ID: 0.0,
        SIDE_MARKER_POS_X_ID: 0.0,
        SIDE_MARKER_POS_Y_ID: 0.0,
        SIDE_MARKER_NEG_X_ID: 0.0,
        SIDE_MARKER_NEG_Y_ID: 0.0,
    })

    # Out-of-plane recess of the marker face, meters. Default 0.0 means the
    # marker plane coincides with the CAD outer surface (d = 29.5mm).
    # The printed cube (_cube_print/..._recess0p1...) has a 0.1mm recess pocket,
    # but paper/sticker thickness usually brings the tag flush with the surface,
    # so 0.0 is the physically correct default. Set to 0.0001 only if you have
    # verified the tags actually sit 0.1mm below the surface.
    marker_inset_m: float = 0.0

    # Optional explicit marker pose override: marker_id -> 4x4 T_object_marker.
    marker_pose_4x4: Dict[int, list] = field(default_factory=dict)


def get_default_cube_config() -> CubeConfig:
    return CubeConfig()


def get_default_cube_config_source() -> str:
    return "config_py:CubeConfig"


@dataclass
class CharucoBoardConfig:
    squares_x: int = 11
    squares_y: int = 7
    square_length_m: float = 0.025
    marker_length_m: float = 0.018
    dictionary_name: str = "DICT_4X4_250"
    marker_id_start: int = 5  # 인쇄된 보드의 ArUco ID 시작값. 큐브(DICT_APRILTAG_36h11)와 다른 딕셔너리(DICT_4X4_250)라 ID 겹쳐도 무방
