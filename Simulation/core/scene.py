"""
합성 씬 — GT 변환을 정하고 검증된 체인으로 관측을 역산 생성 (렌더링 없음).

생성물 (모든 방식이 공유; 각 실험이 필요한 마커만 골라 씀):
  고정 카메라(eye-to-hand) N대, 그리퍼 카메라(eye-in-hand) 1대.
  타깃: 큐브(set 마다 재배치, 로봇이 FK 로 위치 앎) + 보드(테이블 고정, FK 없음).

관측 (camera←target, 노이즈 포함):
  obs_fix_cube[(ci,s)] = inv(bTf[ci]) @ bTo[s]
  obs_fix_board[(ci,s)]= inv(bTf[ci]) @ bTboard
  obs_grip_cube[e]     = inv(bTg[e] @ gTc) @ bTo[s(e)]
  obs_grip_board[e]    = inv(bTg[e] @ gTc) @ bTboard

노이즈:
  systematic : 큐브/보드 base 위치 (x,y) 에 선형 의존하는 위치의존 편향
               (렌즈왜곡·intrinsic 잔차·작업공간 휨 등 실제 검출오차의 지배성분)
  gaussian   : 매 관측 독립 가우시안 (옵션, σ_mm)
  fk_noise   : 로봇 FK 큐브 prior(fk_cube)에 SE(3) 섭동 (Fig B: FK 부정확 모델)

주의: pose-level 시뮬. 코너(px) 노이즈·재투영(px)은 corner-level 확장에서 지원(추후).
"""
import numpy as np
from .se3 import inv_T, rand_se3, rot_axis_angle, look_at


class SimScene:
    def __init__(self, seed=0, n_fixed_cams=3, n_sets=8, n_events_per_set=6,
                 noise_mm=6.0, noise_kind="systematic", gauss_mm=0.0,
                 base_jitter_mm=2.0, fk_noise_mm=0.0, fk_noise_deg=0.0,
                 cam_radius_m=0.45, cam_height_m=0.35, gripper_tilt_deg=35.0,
                 level="pose", sigma_px=0.5):
        rng = np.random.default_rng(seed)
        self.rng = rng
        self.base_jitter_mm = base_jitter_mm
        self.fixed_cam_ids = list(range(n_fixed_cams))
        self.sets = list(range(n_sets))

        # ---- GT 변환 (솔버가 복원해야 할 미지수) ----
        # 핸드아이 gTc: 실물처럼 카메라가 그리퍼 축에 대략 정렬(작은 오프셋). 완전 랜덤이면
        #   eye-in-hand 카메라가 아무 데나 봐서 타깃을 못 봄.
        self.gTc = rand_se3(rng, t_range_m=0.05, ang_range_deg=20.0)     # 핸드아이(소오프셋)
        center = np.zeros(3)
        # 고정 카메라: 작업공간을 원형으로 둘러싸고 중심을 바라봄
        self.bTf = {}
        for k, ci in enumerate(self.fixed_cam_ids):
            th = 2 * np.pi * k / max(n_fixed_cams, 1)
            pos = center + np.array([cam_radius_m * np.cos(th),
                                     cam_radius_m * np.sin(th),
                                     cam_height_m + rng.uniform(-0.02, 0.02)])
            self.bTf[ci] = look_at(pos, center)
        # 보드: 테이블(작업공간)에 고정
        self.bTboard = rand_se3(rng, t_range_m=0.15, ang_range_deg=180.0)
        self.bTboard[:3, 3] = center + np.array([0.0, 0.0, 0.0])
        # 큐브: set 마다 재배치. 실물처럼 "테이블에 앉은" 자세 — 윗면(+Z) 위로, yaw 자유 +
        #   작은 틸트만(뒤집힘 없음). 둘러싼 고정 카메라는 옆면을, 위 그리퍼는 윗면을 봄.
        self.bTo = {}
        for s in self.sets:
            yaw = rng.uniform(-np.pi, np.pi)
            Ryaw = rot_axis_angle(np.array([0, 0, 1.0]), yaw)
            ax = rng.normal(size=3); ax[2] = 0; ax /= (np.linalg.norm(ax) + 1e-12)
            Rtilt = rot_axis_angle(ax, np.deg2rad(rng.uniform(-15, 15)))   # 작은 틸트
            T = np.eye(4)
            T[:3, :3] = Rtilt @ Ryaw
            T[:3, 3] = center + np.array([rng.uniform(-0.1, 0.1),
                                          rng.uniform(-0.1, 0.1),
                                          rng.uniform(0.0, 0.05)])
            self.bTo[s] = T

        # ---- 로봇 그리퍼 자세 bTg (event 마다). 작업공간 위에서 내려다봄 ----
        self.events = []
        self.event_set = {}
        self.bTg = {}
        eid = 0
        for s in self.sets:
            for _ in range(n_events_per_set):
                # 그리퍼 카메라를 작업공간 위쪽에 두고 중심(큐브·보드)을 바라보게 (look-at).
                #   → eye-in-hand 카메라가 실제로 타깃을 관측. bTg 는 그로부터 역산.
                cam_pos = center + np.array([rng.uniform(-0.12, 0.12),
                                             rng.uniform(-0.12, 0.12),
                                             rng.uniform(0.30, 0.45)])
                look = center + rng.uniform(-0.03, 0.03, size=3)   # 중심 근처 약간 흔들림
                cam_pose = look_at(cam_pos, look)
                self.bTg[eid] = cam_pose @ inv_T(self.gTc)         # cam = bTg @ gTc
                self.event_set[eid] = s
                self.events.append(eid)
                eid += 1
        self.set_events = {s: [e for e in self.events if self.event_set[e] == s]
                           for s in self.sets}

        # ---- 로봇이 FK 로 아는 큐브 위치 (fk_cube). 완벽=GT, 옵션으로 노이즈 ----
        self.fk_cube = {}
        for s in self.sets:
            T = self.bTo[s].copy()
            if fk_noise_mm > 0 or fk_noise_deg > 0:
                ax = rng.normal(size=3); ax /= (np.linalg.norm(ax) + 1e-12)
                dR = rot_axis_angle(ax, np.deg2rad(rng.normal(0, fk_noise_deg)))
                T[:3, :3] = dR @ T[:3, :3]
                T[:3, 3] = T[:3, 3] + rng.normal(0, fk_noise_mm / 1000, 3)
            self.fk_cube[s] = T

        # ---- 관측 생성 ----
        self._setup_noise(seed, noise_mm, noise_kind)
        self.noise_kind = noise_kind
        self.gauss_mm = gauss_mm
        self.level = level                      # "pose" | "corner"
        self.sigma_px = sigma_px
        self.obs_fix_cube, self.obs_fix_board = {}, {}
        self.obs_grip_cube, self.obs_grip_board = {}, {}
        self.reproj = {}                        # 관측키 → 재투영오차(px) (corner-level)
        if level == "corner":
            self._gen_corner_obs(seed)
        else:
            self._gen_pose_obs()

    def _gen_pose_obs(self):
        """pose-level 관측: GT pose 에 직접 노이즈(systematic+jitter)."""
        for ci in self.fixed_cam_ids:
            R_cb = inv_T(self.bTf[ci])[:3, :3]
            for s in self.sets:
                self.obs_fix_cube[(ci, s)] = self._obs(
                    inv_T(self.bTf[ci]) @ self.bTo[s], R_cb, self.bTo[s][:2, 3],
                    self.G_fix[ci], (ci, s, "fc"))
                self.obs_fix_board[(ci, s)] = self._obs(
                    inv_T(self.bTf[ci]) @ self.bTboard, R_cb, self.bTboard[:2, 3],
                    self.G_fix[ci], (ci, s, "fb"))
        for e in self.events:
            s = self.event_set[e]
            R_cb = inv_T(self.bTg[e] @ self.gTc)[:3, :3]
            self.obs_grip_cube[e] = self._obs(
                inv_T(self.bTg[e] @ self.gTc) @ self.bTo[s], R_cb,
                self.bTo[s][:2, 3], self.G_grip, ("g", e, "gc"))
            self.obs_grip_board[e] = self._obs(
                inv_T(self.bTg[e] @ self.gTc) @ self.bTboard, R_cb,
                self.bTboard[:2, 3], self.G_grip, ("g", e, "gb"))

    def _gen_corner_obs(self, seed):
        """corner-level 관측: 3D 코너→2D 투영→픽셀노이즈→solvePnP. 실물 마커 기하 반영.
        면 가시성(입사각)으로 큐브 다면성/보드 평면 차이가 자연 발생. 미검출은 관측 없음."""
        from .targets import CubeTarget, BoardTarget
        from .project import observe
        cube, board = CubeTarget(), BoardTarget()
        rng = np.random.default_rng(7000 + seed)
        for ci in self.fixed_cam_ids:
            for s in self.sets:
                self._obs_corner(cube, inv_T(self.bTf[ci]) @ self.bTo[s],
                                 self.obs_fix_cube, (ci, s), rng)
                self._obs_corner(board, inv_T(self.bTf[ci]) @ self.bTboard,
                                 self.obs_fix_board, (ci, s), rng)
        for e in self.events:
            s = self.event_set[e]
            base_g = inv_T(self.bTg[e] @ self.gTc)
            self._obs_corner(cube, base_g @ self.bTo[s], self.obs_grip_cube, e, rng)
            self._obs_corner(board, base_g @ self.bTboard, self.obs_grip_board, e, rng)

    def _obs_corner(self, target, T_gt, store, key, rng):
        from .project import observe
        r = observe(target, T_gt, sigma_px=self.sigma_px, rng=rng)
        if r is not None:
            T_est, ncorner, reproj = r
            store[key] = T_est
            self.reproj[(str(store is self.obs_fix_board or store is self.obs_grip_board),
                         str(key))] = reproj

    # ------------------------------------------------------------------
    def _setup_noise(self, seed, noise_mm, noise_kind):
        """systematic 편향 행렬 G (base 위치 (x,y) → base 편향벡터). 공통+센서별."""
        rb = np.random.default_rng(1000 + seed)
        s_pos = noise_mm / 1000 / 0.3
        self.noise_mm = noise_mm
        self.G_common = rb.normal(0, s_pos * 0.7, (3, 2))
        self.G_fix = {ci: rb.normal(0, s_pos * 0.7, (3, 2)) for ci in self.fixed_cam_ids}
        self.G_grip = rb.normal(0, s_pos * 0.7, (3, 2))

    def _obs(self, T_clean, R_cb, xy, G_sensor, key):
        """관측(camera←target)에 노이즈 주입.
        두 성분:
          systematic : 타깃 base 위치 (x,y) 에 선형 의존하는 편향 (움직이는 큐브에서 주로
                       나타남; 고정 보드는 (x,y) 상수라 사실상 상수→흡수됨). FK 후보정이 학습.
          jitter     : 매 관측 독립 가우시안 검출 지터 (보드·큐브 공통, 항상 존재).
        """
        T = T_clean.copy()
        if self.noise_kind == "systematic":
            bias_base = (self.G_common + G_sensor) @ xy      # base 프레임 위치의존 편향
            T[:3, 3] = T[:3, 3] + R_cb @ bias_base
        elif self.noise_kind == "gaussian":
            rng = np.random.default_rng(hash(key) % (2**31))
            T[:3, 3] = T[:3, 3] + rng.normal(0, self.noise_mm / 1000, 3)
        # baseline 검출 지터 (항상) — 보드도 현실적 노이즈를 갖게 함
        jit = max(self.base_jitter_mm, self.gauss_mm)
        if jit > 0:
            rng = np.random.default_rng((hash(key) ^ 0x9e3779b9) % (2**31))
            T[:3, 3] = T[:3, 3] + rng.normal(0, jit / 1000, 3)
        return T
