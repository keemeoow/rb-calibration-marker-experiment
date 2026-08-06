"""
합성 씬 (코너 수준) — GT 변환을 정하고, 실물 마커 3D 코너를 2D 투영→픽셀노이즈→solvePnP
로 관측 pose 를 생성한다. 렌더링 없이 실물 마커 기하(AprilTag 큐브 6면 크기·roll,
ChArUco 보드 11×7)와 실측 카메라 K/왜곡을 반영. 노이즈는 코너 픽셀에서 전파됨.

생성물:
  고정 카메라(eye-to-hand) N대, 그리퍼 카메라(eye-in-hand) 1대.
  타깃: 큐브(set 마다 재배치, 로봇이 FK 로 위치 앎) + 보드(테이블 고정, FK 없음).

관측 (camera←target, solvePnP 복원):
  obs_fix_cube[(ci,s)], obs_fix_board[(ci,s)], obs_grip_cube[e], obs_grip_board[e]
  면 가시성(입사각)으로 미검출이면 관측 없음 → 큐브 다면성/보드 평면 차이 자연 발생.
  self.reproj[...] : 관측별 재투영 오차(px).

노이즈:
  sigma_px  : 코너 검출 픽셀 노이즈 (실측 rms ~0.3px). 주 노이즈원.
  fk_noise  : 로봇 FK 큐브 prior 에 SE(3) 섭동 (Fig B: FK 부정확 모델).
  (위치의존 systematic 편향은 렌즈 왜곡·시야각에서 자연 발생 — 별도 주입 불필요.)
"""
import os
import numpy as np
from .se3 import inv_T, rand_se3, rot_axis_angle, look_at
from .targets import CubeTarget, BoardTarget
from .project import observe, DEFAULT_K


# 모든 씬이 공유하는 타깃 기하 (한 번만 생성)
_CUBE = CubeTarget()
_BOARD = BoardTarget()

# 실측 카메라 배치 (CP_result/C1 에서 추출, 높이 ~0.2m 거의 수평 하향 10°)
_REAL = np.load(os.path.join(os.path.dirname(__file__), "real_setup", "real_cameras.npz"))
_REAL_BTF = _REAL["bTf"]           # (3,4,4) base←camera
_REAL_CENTER = _REAL["center"]     # 작업공간(큐브) 중심


class SimScene:
    def __init__(self, seed=0, n_fixed_cams=3, n_sets=8, n_events_per_set=6,
                 sigma_px=0.3, fk_noise_mm=0.0, fk_noise_deg=0.0,
                 intrinsic_err=0.0, outlier_rate=0.0,
                 cam_radius_m=0.35, cam_height_m=0.35,
                 incidence_max_deg=75.0, use_real_cameras=True,
                 cam_downtilt_deg=27.0, n_gripped_events=0, robust_pnp=True):
        rng = np.random.default_rng(seed)
        self.rng = rng
        self.sigma_px = sigma_px
        self.incidence_max_deg = incidence_max_deg
        self.outlier_rate = outlier_rate
        self.intrinsic_err = intrinsic_err
        # robust_pnp 는 씬(프론트엔드) 속성 → 모든 캘리브 방법이 동일 조건에서 비교됨.
        self.robust_pnp = bool(robust_pnp)
        self.fk_noise_mm = float(fk_noise_mm)
        self.fk_noise_deg = float(fk_noise_deg)

        # ---- 고정 카메라 배치 ----
        # use_real_cameras=True: 실측 위치(높이 ~0.2m) 사용. 단, 저장된 캘리브 행렬의
        #   하향각(9~16°)은 현재 실제 셋업(25~30° 내려봄, 보드가 보이는 정도)과 달라,
        #   실측 카메라 *위치*는 유지하고 광축이 작업공간 중심을 향하되 하향각을
        #   cam_downtilt_deg(기본 27°)로 맞춘다 → 평면 보드가 부분적으로 보임.
        # False: 이상적 원형 look-at 배치 (개발/디버그용).
        if use_real_cameras:
            center = _REAL_CENTER.copy()
            self.fixed_cam_ids = list(range(len(_REAL_BTF)))
            self.bTf = {}
            for ci in self.fixed_cam_ids:
                pos = _REAL_BTF[ci][:3, 3].copy()        # 실측 카메라 위치
                # 광축을 중심으로 향하되, 하향각을 cam_downtilt_deg 로 강제:
                #   수평 방향(중심 향함) + 아래로 tilt 만큼 내림.
                horiz = center - pos; horiz[2] = 0
                horiz = horiz / (np.linalg.norm(horiz) + 1e-12)
                td = np.deg2rad(cam_downtilt_deg)
                aim_dir = np.array([horiz[0] * np.cos(td), horiz[1] * np.cos(td),
                                    -np.sin(td)])         # 아래로 tilt 한 시선
                self.bTf[ci] = look_at(pos, pos + aim_dir)
        else:
            self.fixed_cam_ids = list(range(n_fixed_cams))
            self.bTf = {}
            center = np.zeros(3)
            for k, ci in enumerate(self.fixed_cam_ids):
                th = 2 * np.pi * k / max(n_fixed_cams, 1)
                pos = center + np.array([cam_radius_m * np.cos(th),
                                         cam_radius_m * np.sin(th),
                                         cam_height_m + rng.uniform(-0.02, 0.02)])
                self.bTf[ci] = look_at(pos, center)
        self.sets = list(range(n_sets))

        # 핸드아이 gTc: 카메라가 그리퍼 축에 대략 정렬(작은 오프셋).
        self.gTc = rand_se3(rng, t_range_m=0.05, ang_range_deg=20.0)
        # 보드: 테이블에 고정 (윗면 +Z 위로)
        self.bTboard = np.eye(4)
        self.bTboard[:3, 3] = center.copy()
        # 큐브: set 마다 재배치. 실물처럼 "테이블에 앉은" 자세(윗면 위, yaw 자유 + 작은 틸트).
        self.bTo = {}
        for s in self.sets:
            yaw = rng.uniform(-np.pi, np.pi)
            Ryaw = rot_axis_angle(np.array([0, 0, 1.0]), yaw)
            ax = rng.normal(size=3); ax[2] = 0; ax /= (np.linalg.norm(ax) + 1e-12)
            Rtilt = rot_axis_angle(ax, np.deg2rad(rng.uniform(-15, 15)))
            T = np.eye(4)
            T[:3, :3] = Rtilt @ Ryaw
            T[:3, 3] = center + np.array([rng.uniform(-0.1, 0.1),
                                          rng.uniform(-0.1, 0.1),
                                          rng.uniform(0.0, 0.05)])
            self.bTo[s] = T

        # ---- 로봇 그리퍼 자세 bTg (event 마다). 카메라가 중심을 바라보게 look-at → bTg 역산 ----
        self.events, self.event_set, self.bTg = [], {}, {}
        eid = 0
        for s in self.sets:
            for _ in range(n_events_per_set):
                cam_pos = center + np.array([rng.uniform(-0.12, 0.12),
                                             rng.uniform(-0.12, 0.12),
                                             rng.uniform(0.30, 0.45)])
                look = center + rng.uniform(-0.03, 0.03, size=3)
                self.bTg[eid] = look_at(cam_pos, look) @ inv_T(self.gTc)
                self.event_set[eid] = s
                self.events.append(eid)
                eid += 1
        self.set_events = {s: [e for e in self.events if self.event_set[e] == s]
                           for s in self.sets}

        # ---- 로봇 FK 큐브 위치 (fk_cube). 완벽=GT, 옵션 노이즈(Fig B) ----
        self.fk_cube = {}
        for s in self.sets:
            T = self.bTo[s].copy()
            if fk_noise_mm > 0 or fk_noise_deg > 0:
                ax = rng.normal(size=3); ax /= (np.linalg.norm(ax) + 1e-12)
                dR = rot_axis_angle(ax, np.deg2rad(rng.normal(0, fk_noise_deg)))
                T[:3, :3] = dR @ T[:3, :3]
                T[:3, 3] = T[:3, 3] + rng.normal(0, fk_noise_mm / 1000, 3)
            self.fk_cube[s] = T

        # ---- 카메라별 부정확 intrinsic (K_pnp) — intrinsic 캘리브 오차 모델 ----
        #   참값 DEFAULT_K/DIST 로 투영하지만 PnP 는 카메라마다 고정 섭동된 K_pnp 를 씀
        #   → 위치의존 systematic 편향(FK 후보정이 학습 가능). 그리퍼는 키 'g'.
        self.K_pnp = {}
        rk = np.random.default_rng(9000 + seed)
        cam_keys = list(self.fixed_cam_ids) + ["g"]
        for ck in cam_keys:
            Kp = DEFAULT_K.copy()
            if intrinsic_err > 0:
                # 초점거리·주점을 상대오차(intrinsic_err)만큼 섭동
                Kp[0, 0] *= 1 + rk.normal(0, intrinsic_err)
                Kp[1, 1] *= 1 + rk.normal(0, intrinsic_err)
                Kp[0, 2] += rk.normal(0, intrinsic_err * 100)   # 주점 px
                Kp[1, 2] += rk.normal(0, intrinsic_err * 100)
            self.K_pnp[ck] = Kp

        # ---- 코너 수준 관측 생성 ----
        #   obs_* : PnP pose (캘리브 입력).  raw_* : 같은 키의 원본 2D 코너 관측 dict
        #   (재투영 평가용 — 방법별로 재생성하지 않고 씬이 한 번만 만들어 공유 → 공정).
        self.obs_fix_cube, self.obs_fix_board = {}, {}
        self.obs_grip_cube, self.obs_grip_board = {}, {}
        self.raw_fix_cube, self.raw_fix_board = {}, {}
        self.raw_grip_cube, self.raw_grip_board = {}, {}
        self.reproj = {}
        orng = np.random.default_rng(7000 + seed)
        for ci in self.fixed_cam_ids:
            for s in self.sets:
                self._obs(_CUBE, inv_T(self.bTf[ci]) @ self.bTo[s],
                          self.obs_fix_cube, (ci, s), orng, ci, self.raw_fix_cube)
                self._obs(_BOARD, inv_T(self.bTf[ci]) @ self.bTboard,
                          self.obs_fix_board, (ci, s), orng, ci, self.raw_fix_board)
        for e in self.events:
            s = self.event_set[e]
            base_g = inv_T(self.bTg[e] @ self.gTc)
            self._obs(_CUBE, base_g @ self.bTo[s], self.obs_grip_cube, e, orng, "g",
                      self.raw_grip_cube)
            self._obs(_BOARD, base_g @ self.bTboard, self.obs_grip_board, e, orng, "g",
                      self.raw_grip_board)

        # ---- gripped 캡처: 로봇이 큐브를 들고 넓게 회전, 고정 카메라가 관측 ----
        #   고전적 eye-to-hand: 큐브가 그리퍼에 강체(gripper_cube_X)로 붙어 이동.
        #   큐브 pose(base) = bTg_grip @ X. 고정 카메라만 관측(그리퍼 카메라는 타깃과 함께
        #   움직여 퇴화 → 미사용). 이 관측이 고정 카메라를 로봇 모션으로 base 에 앵커한다
        #   (FK 큐브 prior 에 덜 의존). 실제 세션의 cube_gripped 캡처에 대응.
        self.gripper_cube_X = rand_se3(rng, t_range_m=0.04, ang_range_deg=25.0)  # 진값(미지, 추정대상)
        self.gripped_events = []
        self.bTg_grip = {}
        self.cube_grip = {}                              # ge -> 큐브 pose(base) 진값
        self.obs_fix_cube_grip = {}                      # (ci, ge) -> 고정카메라 관측
        self.raw_fix_cube_grip = {}
        geid = 10000
        for _ in range(int(n_gripped_events)):
            Tc = np.eye(4)                               # 큐브를 들어 넓게 회전(여러 면 노출)
            ax = rng.normal(size=3); ax /= (np.linalg.norm(ax) + 1e-12)
            Tc[:3, :3] = rot_axis_angle(ax, rng.uniform(0, np.pi))
            Tc[:3, 3] = center + np.array([rng.uniform(-0.10, 0.10),
                                           rng.uniform(-0.10, 0.10),
                                           rng.uniform(0.03, 0.13)])
            self.cube_grip[geid] = Tc
            self.bTg_grip[geid] = Tc @ inv_T(self.gripper_cube_X)   # bTg = cube @ inv(X)
            self.gripped_events.append(geid)
            for ci in self.fixed_cam_ids:
                self._obs(_CUBE, inv_T(self.bTf[ci]) @ Tc,
                          self.obs_fix_cube_grip, (ci, geid), orng, ci,
                          self.raw_fix_cube_grip)
            geid += 1

    def _obs(self, target, T_gt, store, key, rng, cam_key, raw_store=None):
        """3D 코너 투영→픽셀노이즈(+outlier)→robust PnP(부정확 K_pnp). 미검출이면 저장 안 함.
        raw_store 에는 원본 2D 코너 관측 dict 을 같은 키로 보관(재투영 평가용)."""
        r = observe(target, T_gt, sigma_px=self.sigma_px,
                    incidence_max_deg=self.incidence_max_deg, rng=rng,
                    K_pnp=self.K_pnp[cam_key], outlier_rate=self.outlier_rate,
                    robust_pnp=self.robust_pnp)
        if r is not None:
            store[key] = r["T"]
            if raw_store is not None:
                raw_store[key] = r
            self.reproj[(id(store), key)] = r["reproj_px"]
