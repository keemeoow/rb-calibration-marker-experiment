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
from .project import observe, DEFAULT_K, REAL_CAM_INTR


# 모든 씬이 공유하는 타깃 기하 (한 번만 생성)
_CUBE = CubeTarget()
_BOARD = BoardTarget()

# 실측 카메라 배치 (CP_result/C1 에서 추출, 높이 ~0.2m 거의 수평 하향 10°)
_REAL = np.load(os.path.join(os.path.dirname(__file__), "real_setup", "real_cameras.npz"))
_LAYOUT_PATH = os.path.join(os.path.dirname(__file__), "real_setup",
                            "real_layout_session02.npz")
_REAL_BTF = _REAL["bTf"]           # (3,4,4) base←camera
_REAL_CENTER = _REAL["center"]     # 작업공간(큐브) 중심


class SimScene:
    def __init__(self, seed=0, n_fixed_cams=3, n_sets=8, n_events_per_set=6,
                 sigma_px=0.3, fk_noise_mm=0.0, fk_noise_deg=0.0,
                 fk_sys_mm=0.0, fk_sys_deg=0.0,
                 intrinsic_err=0.0, outlier_rate=0.0, outlier_px=15.0,
                 fk_slip_sets=0, fk_slip_mm=0.0, fk_slip_deg=0.0,
                 fk_sys_res_ratio=0.4,
                 corner_bias_px=0.0, outlier_focus_cam=None,
                 max_cams_per_set=None,
                 intrinsic_jitter=0.0, use_real_layout=False,
                 cam_radius_m=0.35, cam_height_m=0.35,
                 incidence_max_deg=75.0, use_real_cameras=True,
                 cam_downtilt_deg=27.0, n_gripped_events=0):
        rng = np.random.default_rng(seed)
        self.rng = rng
        self.sigma_px = sigma_px
        self.incidence_max_deg = incidence_max_deg
        self.outlier_rate = outlier_rate
        self.outlier_px = outlier_px
        self.intrinsic_err = intrinsic_err
        # 코너 검출 계통오차: 카메라마다 고정된 픽셀 편향(방향은 카메라별 무작위, 크기 동일)
        self.corner_bias_px = float(corner_bias_px)
        # 이상치 계통오차: 한 카메라에만 이상치가 몰리는 경우
        self.outlier_focus_cam = outlier_focus_cam
        # 내부파라미터 랜덤오차: 프레임마다 초점거리가 흔들림
        self.intrinsic_jitter = float(intrinsic_jitter)
        # 세트마다 몇 대의 고정 카메라가 보는지 제한(현장에서 가림이 있는 경우)
        self.max_cams_per_set = max_cams_per_set

        # ---- 고정 카메라 배치 ----
        # use_real_cameras=True: 실측 위치(높이 ~0.2m) 사용. 단, 저장된 캘리브 행렬의
        #   하향각(9~16°)은 현재 실제 셋업(25~30° 내려봄, 보드가 보이는 정도)과 달라,
        #   실측 카메라 *위치*는 유지하고 광축이 작업공간 중심을 향하되 하향각을
        #   cam_downtilt_deg(기본 27°)로 맞춘다 → 평면 보드가 부분적으로 보임.
        # False: 이상적 원형 look-at 배치 (개발/디버그용).
        if use_real_layout:
            # ── session02 실측 배치를 참값으로 사용 ──────────────
            #   카메라 자세, 큐브 자세, 로봇 이동을 전부 실제 값으로 심는다.
            #   출처: Step3_calibration.py 를 session02 에 돌린 결과.
            L = np.load(_LAYOUT_PATH)
            self.real_layout = True
            self.fixed_cam_ids = [int(c) for c in L["cam_ids"]]
            self.bTf = {ci: L["bTf"][i].copy()
                        for i, ci in enumerate(self.fixed_cam_ids)}
            self.gTc = L["gTc"].copy()
            self.sets = [int(x) for x in L["sets"]]
            self.bTo = {int(s): L["bTo"][i].copy()
                        for i, s in enumerate(self.sets)}
            center = L["bTo"][:, :3, 3].mean(axis=0)
            self.bTboard = np.eye(4)
            self.bTboard[:3, 3] = center.copy()
            self.events, self.event_set, self.bTg = [], {}, {}
            for e, (M, s) in enumerate(zip(L["bTg"], L["event_set"])):
                self.bTg[e] = M.copy()
                self.event_set[e] = int(s)
                self.events.append(e)
            self.set_events = {s: [e for e in self.events if self.event_set[e] == s]
                               for s in self.sets}
        elif use_real_cameras:
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
        if not use_real_layout:
            self.sets = list(range(n_sets))

        # 핸드아이 gTc: 카메라가 그리퍼 축에 대략 정렬(작은 오프셋).
        if not use_real_layout:
            self.gTc = rand_se3(rng, t_range_m=0.05, ang_range_deg=20.0)
        # 보드: 테이블에 고정 (윗면 +Z 위로)
        if not use_real_layout:
            self.bTboard = np.eye(4)
            self.bTboard[:3, 3] = center.copy()
        # 큐브: set 마다 재배치. 실물처럼 "테이블에 앉은" 자세(윗면 위, yaw 자유 + 작은 틸트).
        if not use_real_layout:
            self.bTo = {}
            for s in self.sets:
                yaw = rng.uniform(-np.pi, np.pi)
                Ryaw = rot_axis_angle(np.array([0, 0, 1.0]), yaw)
                ax = rng.normal(size=3); ax[2] = 0
                ax /= (np.linalg.norm(ax) + 1e-12)
                Rtilt = rot_axis_angle(ax, np.deg2rad(rng.uniform(-15, 15)))
                T = np.eye(4)
                T[:3, :3] = Rtilt @ Ryaw
                T[:3, 3] = center + np.array([rng.uniform(-0.1, 0.1),
                                              rng.uniform(-0.1, 0.1),
                                              rng.uniform(0.0, 0.05)])
                self.bTo[s] = T

        # ---- 로봇 그리퍼 자세 bTg (event 마다). 카메라가 중심을 바라보게 look-at → bTg 역산 ----
        if not use_real_layout:
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

        # ---- 로봇 FK 큐브 위치 (fk_cube). 완벽=GT, 옵션 노이즈 ----
        #   두 종류:
        #   (1) fk_sys_*  : systematic 편향 (실측 FK 성격) = **상수 rigid 오정렬**(모든 set 동일:
        #       real 의 180° flip + 37mm 오프셋 대응) + **소량 per-set 잔차**. 상수부는 vision 으로
        #       추정한 T_delta_avg 로 de-bias 하면 제거됨(=Ours corr). 잔차는 남음(실측 dt 1~6mm).
        #   (2) fk_noise_*: random(제로평균) 섭동 — 학습·de-bias 불가한 순수 잡음 (대조 sweep).
        fk_sys = None
        if fk_sys_mm > 0 or fk_sys_deg > 0:
            rs = np.random.default_rng(5000 + seed)
            bvec = rs.normal(size=3); bvec /= (np.linalg.norm(bvec) + 1e-12)
            ax_s = rs.normal(size=3); ax_s /= (np.linalg.norm(ax_s) + 1e-12)
            # Step3 assumes raw_fk @ T_delta == visual for every set. Generate
            # systematic error in exactly that right-multiplied local-frame form.
            T_delta = np.eye(4)
            T_delta[:3, 3] = bvec * (fk_sys_mm / 1000.0)
            T_delta[:3, :3] = rot_axis_angle(ax_s, np.deg2rad(fk_sys_deg))
            # per-set 잔차 std (de-bias 후 남음). 실측 session02 는 계통 40.9mm 에
            # 잔차 3.5mm 로 비율이 0.086 이었다. 기본 0.4 는 예전 가정값이다.
            res_std = (fk_sys_mm / 1000.0) * float(fk_sys_res_ratio)
            fk_sys = (T_delta, res_std)
        self.fk_cube = {}
        for s in self.sets:
            T = self.bTo[s].copy()
            if fk_sys is not None:                             # systematic = 상수 + per-set 잔차
                T_delta, res_std = fk_sys
                T = T @ inv_T(T_delta)                         # raw @ delta = true
                T[:3, 3] = T[:3, 3] + rng.normal(0, res_std, 3)   # per-set 잔차 (제거 불가)
            if fk_noise_mm > 0 or fk_noise_deg > 0:            # random (제로평균)
                ax = rng.normal(size=3); ax /= (np.linalg.norm(ax) + 1e-12)
                dR = rot_axis_angle(ax, np.deg2rad(rng.normal(0, fk_noise_deg)))
                T[:3, :3] = dR @ T[:3, :3]
                T[:3, 3] = T[:3, 3] + rng.normal(0, fk_noise_mm / 1000, 3)
            self.fk_cube[s] = T

        # ---- 특정 세트만 크게 어긋나는 경우 (큐브가 그리퍼 안에서 미끄러짐) ----
        #   상수 de-bias 로는 제거되지 않고 그 세트에만 남는 오차. gate 가 잡아내야 하는 대상.
        self.fk_slip_sets = []
        if fk_slip_sets > 0 and (fk_slip_mm > 0 or fk_slip_deg > 0):
            rslip = np.random.default_rng(9000 + seed)
            pick = rslip.choice(len(self.sets),
                                size=min(int(fk_slip_sets), len(self.sets)),
                                replace=False)
            self.fk_slip_sets = sorted(int(self.sets[i]) for i in pick)
            for s in self.fk_slip_sets:
                ax = rslip.normal(size=3); ax /= (np.linalg.norm(ax) + 1e-12)
                d = rslip.normal(size=3); d /= (np.linalg.norm(d) + 1e-12)
                T = self.fk_cube[s].copy()
                T[:3, :3] = rot_axis_angle(ax, np.deg2rad(fk_slip_deg)) @ T[:3, :3]
                T[:3, 3] = T[:3, 3] + d * (fk_slip_mm / 1000.0)
                self.fk_cube[s] = T

        # ---- 카메라별 개별 실측 intrinsic ----
        #   real 은 카메라마다 K 가 다름(평균 하나 아님). sim 고정캠 0/1/2 → real cam 0/1/3,
        #   그리퍼('g') → real cam2 (REAL_CAM_INTR). 참값(K_true/dist_true)으로 투영하고,
        #   PnP 는 캘리브 오차(intrinsic_err) 만큼 섭동된 K_pnp/dist_pnp 를 씀.
        SIM_TO_REAL = {0: 0, 1: 1, 2: 3, "g": 2}     # sim 카메라 키 → 실측 cam 인덱스
        self.K_true, self.dist_true = {}, {}
        self.K_pnp, self.dist_pnp = {}, {}
        rk = np.random.default_rng(9000 + seed)
        cam_keys = list(self.fixed_cam_ids) + ["g"]
        for ck in cam_keys:
            ri = SIM_TO_REAL.get(ck, 0)
            Kt = REAL_CAM_INTR[ri]["K"].copy()           # 참값 = 실측 개별 K
            dt = REAL_CAM_INTR[ri]["dist"].copy()
            self.K_true[ck] = Kt; self.dist_true[ck] = dt
            Kp = Kt.copy()
            if intrinsic_err > 0:                        # 캘리브 오차(참값 대비 섭동)
                Kp[0, 0] *= 1 + rk.normal(0, intrinsic_err)
                Kp[1, 1] *= 1 + rk.normal(0, intrinsic_err)
                Kp[0, 2] += rk.normal(0, intrinsic_err * 100)
                Kp[1, 2] += rk.normal(0, intrinsic_err * 100)
            self.K_pnp[ck] = Kp
            self.dist_pnp[ck] = dt.copy()                # dist 는 실측 그대로(계측 신뢰)

        # ---- 코너 검출 계통 편향: 카메라마다 방향은 다르고 크기는 같다 ----
        self._corner_bias = {}
        if self.corner_bias_px > 0:
            rb = np.random.default_rng(11000 + seed)
            for ck in cam_keys:
                th = rb.uniform(0, 2 * np.pi)
                self._corner_bias[ck] = (self.corner_bias_px * np.cos(th),
                                         self.corner_bias_px * np.sin(th))

        # ---- 코너 수준 관측 생성 ----
        self.obs_fix_cube, self.obs_fix_board = {}, {}
        self.obs_grip_cube, self.obs_grip_board = {}, {}
        self.reproj = {}
        self.corn = {}      # (id(store), key) -> (obj_rig3d, img2d_noisy, cam_key) : held-out 픽셀 재투영(③)용
        orng = np.random.default_rng(7000 + seed)
        # 세트별로 관측하는 고정 카메라를 제한할 수 있다.
        vis_rng = np.random.default_rng(13000 + seed)
        cams_for_set = {}
        for s in self.sets:
            if self.max_cams_per_set is None:
                cams_for_set[s] = list(self.fixed_cam_ids)
            else:
                k = min(int(self.max_cams_per_set), len(self.fixed_cam_ids))
                pick = vis_rng.choice(len(self.fixed_cam_ids), size=k, replace=False)
                cams_for_set[s] = [self.fixed_cam_ids[i] for i in sorted(pick)]
        self.cams_for_set = cams_for_set

        for ci in self.fixed_cam_ids:
            for s in self.sets:
                if ci not in cams_for_set[s]:
                    continue
                self._obs(_CUBE, inv_T(self.bTf[ci]) @ self.bTo[s],
                          self.obs_fix_cube, (ci, s), orng, ci)
                self._obs(_BOARD, inv_T(self.bTf[ci]) @ self.bTboard,
                          self.obs_fix_board, (ci, s), orng, ci)
        for e in self.events:
            s = self.event_set[e]
            base_g = inv_T(self.bTg[e] @ self.gTc)
            self._obs(_CUBE, base_g @ self.bTo[s], self.obs_grip_cube, e, orng, "g")
            self._obs(_BOARD, base_g @ self.bTboard, self.obs_grip_board, e, orng, "g")

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
                          self.obs_fix_cube_grip, (ci, geid), orng, ci)
            geid += 1

    def _obs(self, target, T_gt, store, key, rng, cam_key):
        """3D 코너 투영→픽셀노이즈(+outlier)→PnP(부정확 K_pnp). 미검출이면 저장 안 함."""
        # 이상치 계통오차: 지정 카메라에만 몰아준다(다른 카메라는 이상치 없음).
        orate = self.outlier_rate
        if self.outlier_focus_cam is not None:
            orate = self.outlier_rate if cam_key == self.outlier_focus_cam else 0.0
        # 내부파라미터 랜덤오차: PnP 가 쓰는 K 를 프레임마다 흔든다.
        K_pnp = self.K_pnp[cam_key]
        if self.intrinsic_jitter > 0:
            K_pnp = K_pnp.copy()
            K_pnp[0, 0] *= 1 + rng.normal(0, self.intrinsic_jitter)
            K_pnp[1, 1] *= 1 + rng.normal(0, self.intrinsic_jitter)
        r = observe(target, T_gt, sigma_px=self.sigma_px,
                    incidence_max_deg=self.incidence_max_deg, rng=rng,
                    K=self.K_true[cam_key], dist=self.dist_true[cam_key],
                    K_pnp=K_pnp, dist_pnp=self.dist_pnp[cam_key],
                    outlier_rate=orate, outlier_px=self.outlier_px,
                    corner_bias_px=self._corner_bias.get(cam_key, (0.0, 0.0)))
        if r is not None:
            T_est, ncorner, reproj, obj, img = r
            store[key] = T_est
            self.reproj[(id(store), key)] = reproj
            self.corn[(id(store), key)] = (obj, img, cam_key)
