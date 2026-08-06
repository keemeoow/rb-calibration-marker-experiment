"""
타깃 3D 코너 정의 — 실물 셋업 반영 (렌더링 없이 코너 수준 시뮬용).

  AprilTag 큐브 : 6개 마커(id 0..5), 면 +Z(2개)·+X·+Y·-X·-Y, 마커 51mm, 면별 roll.
  ChArUco 보드  : 11×7 사각형, 25mm. 내부 체스판 코너를 3D 점으로.

각 마커/코너는 타깃 rig 로컬 좌표에 정의된다. 카메라가 이 3D 점을 K/왜곡으로 2D 투영하고,
픽셀 노이즈를 준 뒤 solvePnP 로 pose 를 복원한다(project.py).

면 법선(outward normal)으로 입사각을 판정 → 카메라를 향한 면만 관측(큐브 다면성 반영).
"""
import numpy as np
from .se3 import rot_axis_angle

# ---- 실물 큐브 config (config.py) ----
#   윗면(+Z, id 0·1) 마커는 25mm, 옆면(id 2~5) 마커는 51mm (실물 크기 다름).
CUBE_TOP_SIZE_M = 0.025
CUBE_SIDE_SIZE_M = 0.051
CUBE_ID_TO_FACE = {0: "+Z", 1: "+Z", 2: "+X", 3: "+Y", 4: "-X", 5: "-Y"}
CUBE_FACE_ROLL_DEG = {0: 0.0, 1: 0.0, 2: 90.0, 3: 180.0, 4: 270.0, 5: 0.0}
CUBE_MARKER_SIZE = {0: CUBE_TOP_SIZE_M, 1: CUBE_TOP_SIZE_M,
                    2: CUBE_SIDE_SIZE_M, 3: CUBE_SIDE_SIZE_M,
                    4: CUBE_SIDE_SIZE_M, 5: CUBE_SIDE_SIZE_M}
CUBE_HALF_M = CUBE_SIDE_SIZE_M / 2.0   # 큐브 반변(면 중심 오프셋). 옆면 크기 기준.

# 면 정의: 이름 -> (면 중심 오프셋 방향 = 법선, 면 내 u축, v축)
_FACE_DEFS = {
    "+Z": (np.array([0, 0, 1.0]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0])),
    "-Z": (np.array([0, 0, -1.0]), np.array([1.0, 0, 0]), np.array([0, -1.0, 0])),
    "+X": (np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])),
    "-X": (np.array([-1.0, 0, 0]), np.array([0, -1.0, 0]), np.array([0, 0, 1.0])),
    "+Y": (np.array([0, 1.0, 0]), np.array([-1.0, 0, 0]), np.array([0, 0, 1.0])),
    "-Y": (np.array([0, -1.0, 0]), np.array([1.0, 0, 0]), np.array([0, 0, 1.0])),
}

# ---- 실물 보드 config (CharucoBoardConfig) ----
BOARD_SQUARES_X = 11
BOARD_SQUARES_Y = 7
BOARD_SQUARE_M = 0.025


def _marker_local_corners(size_m):
    """마커 로컬 평면 4코너 (z=0), 반시계. 중심 원점."""
    h = size_m / 2.0
    return np.array([[-h, -h, 0], [h, -h, 0], [h, h, 0], [-h, h, 0]], float)


class CubeTarget:
    """AprilTag 큐브 — 마커별 3D 코너(rig 좌표) + 면 법선.
    partial_ok=False: AprilTag 는 4코너가 모두 보여야 디코딩된다(마커 단위 all-or-nothing)."""
    partial_ok = False

    def __init__(self):
        self.markers = {}          # id -> {"corners3d":(4,3), "normal":(3,), "center":(3,)}
        for mid, face in CUBE_ID_TO_FACE.items():
            n, u, v = _FACE_DEFS[face]
            roll = np.deg2rad(CUBE_FACE_ROLL_DEG.get(mid, 0.0))
            Rr = rot_axis_angle(n, roll)                        # 면 법선 축 roll
            u2, v2 = Rr @ u, Rr @ v
            center = n * CUBE_HALF_M
            loc = _marker_local_corners(CUBE_MARKER_SIZE[mid])   # 윗면 25 / 옆면 51mm
            # 로컬(u,v,평면) → rig 3D
            corners3d = np.array([center + c[0] * u2 + c[1] * v2 for c in loc])
            self.markers[mid] = {"corners3d": corners3d, "normal": n, "center": center}

    def all_corners(self):
        """(marker_id, corners3d(4,3), normal) 리스트."""
        return [(mid, m["corners3d"], m["normal"]) for mid, m in self.markers.items()]


class BoardTarget:
    """ChArUco 보드 — 내부 체스판 코너 3D(평면 z=0). 법선 +Z 한 면.
    partial_ok=True: 체스판 코너는 개별 검출되므로 화면 안에 든 코너만 부분 사용한다
    (실제 ChArUco 검출과 동일. 전부-아니면-전무로 두면 board-only 비교군이 불리해진다)."""
    partial_ok = True

    def __init__(self):
        sx, sy, sq = BOARD_SQUARES_X, BOARD_SQUARES_Y, BOARD_SQUARE_M
        # 내부 코너: (sx-1) x (sy-1)
        pts = []
        for iy in range(1, sy):
            for ix in range(1, sx):
                pts.append([ix * sq - sx * sq / 2, iy * sq - sy * sq / 2, 0.0])
        self.corners3d = np.array(pts, float)
        self.normal = np.array([0, 0, 1.0])

    def all_corners(self):
        """보드는 한 면 → (0, 전체코너, 법선) 단일."""
        return [(0, self.corners3d, self.normal)]
