"""SE(3) 저수준 유틸 — 자체 완결(부모 저장소 의존 없음). numpy + scipy 만 사용.

명명: 변환은 "목적지←출발지", T_A_C = T_A_B @ T_B_C.
"""
import numpy as np
from scipy.spatial.transform import Rotation


# ---------------------------------------------------------------- 기본 변환
def inv_T(T: np.ndarray) -> np.ndarray:
    """4x4 SE(3) 역변환."""
    R = T[:3, :3]; t = T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def rot_axis_angle(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    """축-각 → 3x3 회전행렬 (Rodrigues)."""
    axis = np.asarray(axis, float)
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    return Rotation.from_rotvec(axis * angle_rad).as_matrix()


def rand_se3(rng: np.random.Generator, t_range_m=0.5, ang_range_deg=180.0) -> np.ndarray:
    """랜덤 SE(3) (랜덤 축-각 회전 + 랜덤 병진)."""
    ax = rng.normal(size=3); ax /= (np.linalg.norm(ax) + 1e-12)
    ang = np.deg2rad(rng.uniform(-ang_range_deg, ang_range_deg))
    T = np.eye(4)
    T[:3, :3] = rot_axis_angle(ax, ang)
    T[:3, 3] = rng.uniform(-t_range_m, t_range_m, size=3)
    return T


def look_at(cam_pos, target, world_up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """cam_pos 에서 target 을 바라보는 bTf (base←camera). OpenCV 규약(+z 光축)."""
    cam_pos = np.asarray(cam_pos, float); target = np.asarray(target, float)
    z = target - cam_pos; z /= (np.linalg.norm(z) + 1e-12)
    up = np.asarray(world_up, float)
    x = np.cross(up, z)
    if np.linalg.norm(x) < 1e-6:
        up = np.array([0.0, 1.0, 0.0]); x = np.cross(up, z)
    x /= (np.linalg.norm(x) + 1e-12)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, y, z])
    T[:3, 3] = cam_pos
    return T


# ---------------------------------------------------------------- BA 파라미터화
def se3_to_vec(T: np.ndarray) -> np.ndarray:
    """SE(3) → 6-vec [rotvec(3), trans(3)]."""
    rv = Rotation.from_matrix(T[:3, :3]).as_rotvec()
    return np.concatenate([rv, T[:3, 3]])


def vec_to_se3(v: np.ndarray) -> np.ndarray:
    """6-vec [rotvec, trans] → SE(3)."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(v[:3]).as_matrix()
    T[:3, 3] = v[3:6]
    return T


def se3_residual(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """두 SE(3) 불일치 6-vec [rotvec(rad), trans(m)]. A==B → 0."""
    E = inv_T(A) @ B
    rv = Rotation.from_matrix(E[:3, :3]).as_rotvec()
    return np.concatenate([rv, E[:3, 3]])


# ---------------------------------------------------------------- 평균 / 오차
def se3_avg(Ts) -> np.ndarray:
    """SE(3) 리스트의 강건 평균 (회전=쿼터니언 평균, 병진=median)."""
    Ts = [np.asarray(T, float) for T in Ts]
    R = Rotation.from_matrix([T[:3, :3] for T in Ts]).mean().as_matrix()
    t = np.median(np.array([T[:3, 3] for T in Ts]), axis=0)
    out = np.eye(4); out[:3, :3] = R; out[:3, 3] = t
    return out


def rot_deg(A: np.ndarray, B: np.ndarray) -> float:
    """두 pose 회전 측지 오차 (deg)."""
    R = A[:3, :3].T @ B[:3, :3]
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    return float(np.degrees(np.arccos(c)))


def trans_mm(A: np.ndarray, B: np.ndarray) -> float:
    """두 pose 병진 오차 (mm)."""
    return float(np.linalg.norm(A[:3, 3] - B[:3, 3]) * 1000)


def fit_rigid(P, Q):
    """P→Q 최소제곱 rigid (R,t): Q ≈ R P + t (Kabsch)."""
    P = np.asarray(P); Q = np.asarray(Q)
    cP, cQ = P.mean(0), Q.mean(0)
    H = (P - cP).T @ (Q - cQ)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R, cQ - R @ cP
