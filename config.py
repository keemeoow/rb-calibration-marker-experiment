# config.py
"""
캘리브레이션 파이프라인 공통 설정.

이 파일의 CubeConfig dataclass가 큐브 모델의 단일 source of truth다.
동일한 물리 큐브를 계속 사용할 경우, 별도 override 없이 이 정의를 그대로 사용한다.
예외적으로 다른 큐브/실험 정의가 필요할 때만 명시적인 JSON override를 사용한다.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import numpy as np


# ════════════════════════════════════════════════════════════════════════════
#  USER-EDITABLE MARKER IDs  ─  AprilTag 59mm cube
# ────────────────────────────────────────────────────────────────────────────
#  If you re-print the cube with different AprilTag IDs, edit ONLY this block.
#  Everything downstream (face mapping, sizes, object-frame centers) is keyed
#  off these constants, so nothing else needs to change.
#
#  Cube net (looking from above, +Z up):
#                 ┌───────────────┐
#                 │  TOP  +Z face │   two small 25mm tags, stacked along Y:
#                 │  [POS_Y_ID]   │     center y = +14mm
#                 │  [NEG_Y_ID]   │     center y = -14mm
#                 └───────────────┘
#   side faces carry one large 51mm tag each (+X, +Y, -X, -Y).
# ════════════════════════════════════════════════════════════════════════════
TOP_MARKER_NEG_Y_ID = 0   # top face, tag whose center is at y = -14mm
TOP_MARKER_POS_Y_ID = 1   # top face, tag whose center is at y = +14mm
SIDE_MARKER_POS_X_ID = 2  # +X side face
SIDE_MARKER_POS_Y_ID = 3  # +Y side face
SIDE_MARKER_NEG_X_ID = 4  # -X side face
SIDE_MARKER_NEG_Y_ID = 5  # -Y side face

# Physical AprilTag sizes (printed black-square outer edge). UNITS: meters (m).
TOP_MARKER_SIZE_M = 0.025   # 25mm  (the two top tags)
SIDE_MARKER_SIZE_M = 0.051  # 51mm  (the four side tags)
# ════════════════════════════════════════════════════════════════════════════


@dataclass
class CubeConfig:
    """AprilTag marker-cube target configuration.

    Physical CAD/measured definition (UNITS in field names: *_m = meters):
      - overall bounding cube : 59 x 59 x 59 mm
      - lower body            : 59 x 59 x 57 mm   (carries the 4 side tags)
      - upper narrow shelf    : 31 x 59 x  2 mm   (carries the 2 top tags)
      - grips (ignored here)  : 10 x  2 x 0.3 mm  on the upper shelf sides
      - recess depth          : 0.1 mm  (see marker_inset_m note below)

    Object-frame convention (kept identical to the legacy ArUco cube):
      - origin at the geometric center of the 59mm bounding cube
      - +Z up, so top face z = +29.5mm, bottom face z = -29.5mm
      - per-marker orientation comes from the face it sits on (face_defs in
        aruco_cube.py) plus face_roll_deg; per-marker CENTER comes from
        marker_center_m so two tags can share one face.
    """
    cube_side_m: float = 0.059          # bounding-cube edge length (m) - 59mm
    # NOTE: top and side tags differ in size, so marker_size_m is only a
    # fallback. Real per-tag sizes live in marker_size_by_id below.
    marker_size_m: float = SIDE_MARKER_SIZE_M   # fallback marker size (m)
    dictionary_name: str = "DICT_APRILTAG_36h11"  # AprilTag 36h11 family
    marker_ids: Tuple[int, ...] = (
        TOP_MARKER_NEG_Y_ID, TOP_MARKER_POS_Y_ID,
        SIDE_MARKER_POS_X_ID, SIDE_MARKER_POS_Y_ID,
        SIDE_MARKER_NEG_X_ID, SIDE_MARKER_NEG_Y_ID,
    )

    # marker_id -> face name (which face the tag's plane lies on)
    id_to_face: Dict[int, str] = field(default_factory=lambda: {
        TOP_MARKER_NEG_Y_ID: "+Z",
        TOP_MARKER_POS_Y_ID: "+Z",
        SIDE_MARKER_POS_X_ID: "+X",
        SIDE_MARKER_POS_Y_ID: "+Y",
        SIDE_MARKER_NEG_X_ID: "-X",
        SIDE_MARKER_NEG_Y_ID: "-Y",
    })

    # Per-marker physical size. UNITS: meters (m). Used to build that marker's
    # 3D corner coordinates (see ArucoCubeModel.marker_corners_in_rig).
    marker_size_by_id: Dict[int, float] = field(default_factory=lambda: {
        TOP_MARKER_NEG_Y_ID: TOP_MARKER_SIZE_M,
        TOP_MARKER_POS_Y_ID: TOP_MARKER_SIZE_M,
        SIDE_MARKER_POS_X_ID: SIDE_MARKER_SIZE_M,
        SIDE_MARKER_POS_Y_ID: SIDE_MARKER_SIZE_M,
        SIDE_MARKER_NEG_X_ID: SIDE_MARKER_SIZE_M,
        SIDE_MARKER_NEG_Y_ID: SIDE_MARKER_SIZE_M,
    })

    # Per-marker CENTER in the cube/object frame. UNITS: meters (m).
    # This overrides the geometric face-center, which is required because:
    #   - the two top tags share the +Z face but sit at y = +/-14mm
    #   - the side tags sit on the 57mm lower body, so their center is at
    #     z = -1mm (lower-body center), NOT z = 0 (full-cube center).
    # Derivation of side z: lower body spans z = -29.5 .. +27.5mm -> center -1mm.
    marker_center_m: Dict[int, Tuple[float, float, float]] = field(default_factory=lambda: {
        # top tags: x=0, z=+29.5mm, 3-25-3-25-3 layout along Y -> y=+/-14mm
        TOP_MARKER_NEG_Y_ID: (0.0, -0.014, 0.0295),
        TOP_MARKER_POS_Y_ID: (0.0,  0.014, 0.0295),
        # side tags: centered on the 57mm lower body -> z = -1mm
        SIDE_MARKER_POS_X_ID: ( 0.0295, 0.0,   -0.001),
        SIDE_MARKER_POS_Y_ID: (0.0,    0.0295, -0.001),
        SIDE_MARKER_NEG_X_ID: (-0.0295, 0.0,   -0.001),
        SIDE_MARKER_NEG_Y_ID: (0.0,   -0.0295, -0.001),
    })

    # Maps detector corner order to each marker's local [0,1,2,3] order.
    # Identity by default; adjust only if a printed tag is rotated/mirrored.
    corner_reorder: Dict[int, list] = field(default_factory=lambda: {
        TOP_MARKER_NEG_Y_ID: [0, 1, 2, 3],
        TOP_MARKER_POS_Y_ID: [0, 1, 2, 3],
        SIDE_MARKER_POS_X_ID: [0, 1, 2, 3],
        SIDE_MARKER_POS_Y_ID: [0, 1, 2, 3],
        SIDE_MARKER_NEG_X_ID: [0, 1, 2, 3],
        SIDE_MARKER_NEG_Y_ID: [0, 1, 2, 3],
    })

    # Per-marker in-plane rotation (deg) about the face normal.
    # NOTE: these depend on how each AprilTag was oriented when printed and
    # must be validated against the physical cube. Defaulting to 0 here.
    face_roll_deg: Dict[int, float] = field(default_factory=lambda: {
        TOP_MARKER_NEG_Y_ID: 0.0,
        TOP_MARKER_POS_Y_ID: 0.0,
        SIDE_MARKER_POS_X_ID: 0.0,
        SIDE_MARKER_POS_Y_ID: 0.0,
        SIDE_MARKER_NEG_X_ID: 0.0,
        SIDE_MARKER_NEG_Y_ID: 0.0,
    })

    # Optional: marker recess / inset along the face inward normal. UNITS: m.
    # The tags are recessed 0.1mm below the CAD surface. For now the marker
    # plane is defined ON the CAD outer surface (inset = 0). To model the
    # recess later, set this > 0 and apply it as a depth correction in
    # ArucoCubeModel.marker_pose_in_rig (currently NOT applied).
    marker_inset_m: float = 0.0

    # Optional explicit rigid pose of each marker in the cube/object frame.
    # When present, this overrides face+center geometry construction for that
    # marker. corner_reorder is still used to map detector corners to the
    # marker's local [0,1,2,3] order.
    marker_pose_4x4: Dict[int, list] = field(default_factory=dict)


def get_default_cube_config() -> CubeConfig:
    """Return a fresh copy of the canonical cube definition (CubeConfig dataclass defaults)."""
    return CubeConfig()


def get_default_cube_config_source() -> str:
    return "config_py:CubeConfig"


@dataclass
class CharucoBoardConfig:
    """ChArUco board target configuration (for eye-in-hand / gripper camera)."""
    squares_x: int = 11           # number of squares in X
    squares_y: int = 7            # number of squares in Y
    square_length_m: float = 0.025   # checker square side (m) - 25mm
    marker_length_m: float = 0.018   # ArUco marker side (m) - 18mm
    dictionary_name: str = "DICT_4X4_250"  # 7x11 board needs ~39 markers
    marker_id_start: int = 5      # reserve cube IDs 0~4


@dataclass
class CameraStreamConfig:
    """RealSense stream config."""
    color_w: int = 640
    color_h: int = 480
    depth_w: int = 640
    depth_h: int = 480
    fps: int = 15


@dataclass
class RobotConfig:
    """Robot communication config."""
    host: str = "192.168.0.23"
    port: int = 12348
    # Euler convention for your robot (ZYX intrinsic = extrinsic XYZ)
    # robot_poses format: [x_mm, y_mm, z_mm, rz_deg, ry_deg, rx_deg]
    euler_order: str = "ZYX"


@dataclass
class CalibrationConfig:
    """Calibration parameters."""
    # ArUco detection
    min_markers: int = 1
    reproj_max_px: float = 10.0
    use_ransac: bool = True

    # Hand-eye
    handeye_method: int = 4   # cv2.CALIB_HAND_EYE_PARK

    # Multi-cam
    ref_fixed_cam_idx: int = 1       # which fixed camera is the reference
    gripper_cam_idx: int = 0         # which cam index is the gripper camera

    # Point cloud fusion
    z_min: float = 0.2
    z_max: float = 1.5
    stride: int = 4
