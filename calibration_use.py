#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""calibration_use.py — 지금 바로 쓰는 캘리브 결과 로더.

카메라가 본 물체 자세(`T_cam_object`)를 로봇 base 좌표로 옮긴다.

    from calibration_use import Calibration
    calib = Calibration()                      # data/session02/calib_final_use 를 읽는다
    p = calib.to_base(cam_id=1, T_cam_object=T)        # 카메라 한 대
    p = calib.fuse({1: T1, 3: T3})                     # 두 대를 합침 (권장)

반환은 base 프레임 위치(미터, 3-vector)다. 자세(회전)까지 필요하면 `to_base_pose` 를
쓰되, 정확도 수치는 위치에 대해서만 측정된 값이다.

**cam0 는 쓰지 않는다.** 단독 오차가 16.8 mm 로 cam1(4.7)·cam3(4.4)의 네 배다.
자세한 근거는 `data/session02/calib_final_use/calibration_for_use.json` 과
`ABLATION_TEST_result/final_fk_mode_fit/FINAL_CHOICE.md` 에 있다.
"""
from __future__ import annotations

import json
import os
import warnings
from typing import Dict, Mapping, Optional, Sequence

import numpy as np

DEFAULT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "data", "session02", "calib_final_use")


class Calibration:
    def __init__(self, calib_dir: str = DEFAULT_DIR, use_ridge: bool = True,
                 robot_tool_offset_z_m: Optional[float] = None):
        """robot_tool_offset_z_m: 지금 로봇에 설정된 툴오프셋(미터).

        캘리브는 툴오프셋 z=150 mm 상태에서 했다. 그리퍼 카메라를 쓸 때만 문제가 되며,
        다른 값을 쓰고 있으면 여기에 알려주면 `T_gripper_cam` 을 그 프레임으로 옮긴다.
        고정 카메라(cam1/cam3)는 base 프레임이라 툴오프셋과 무관하다.
        """
        with open(os.path.join(calib_dir, "calibration_for_use.json")) as handle:
            self.meta = json.load(handle)
        self.dir = calib_dir
        self.T_base_cam: Dict[int, np.ndarray] = {
            int(k.split("C")[-1]): np.asarray(v, float)
            for k, v in self.meta["transforms"].items() if k.startswith("T_base_C")}
        self.T_gripper_cam = np.asarray(self.meta["transforms"]["T_gripper_cam"], float)
        conv = self.meta.get("gripper_cam_tcp_convention", {})
        self.calibration_tool_offset_z = float(conv.get("calibration_tcp_offset_z_m", 0.150))
        self.tool_offset_z = (self.calibration_tool_offset_z
                              if robot_tool_offset_z_m is None
                              else float(robot_tool_offset_z_m))
        if abs(self.tool_offset_z - self.calibration_tool_offset_z) > 1e-9:
            shift = np.eye(4)
            shift[2, 3] = self.calibration_tool_offset_z - self.tool_offset_z
            self.T_gripper_cam = shift @ self.T_gripper_cam
        self.use_cameras = [int(c) for c in self.meta["use_cameras"]]
        self.W = np.asarray(self.meta["ridge_correction"]["W"], float)
        self.use_ridge = bool(use_ridge)

    # ── 좌표 변환 ────────────────────────────────────────────────────────────
    def to_base_pose(self, cam_id: int, T_cam_object, robot_T_base_gripper=None) -> np.ndarray:
        """카메라 좌표계의 물체 자세를 base 프레임 4x4 로 옮긴다.

        고정 카메라면 `T_base_Ci @ T_cam_object`.
        그리퍼 카메라(2)면 로봇의 현재 `T_base_gripper` 가 필요하다.
        """
        T_cam_object = np.asarray(T_cam_object, float)
        cam_id = int(cam_id)
        if cam_id in self.T_base_cam:
            return self.T_base_cam[cam_id] @ T_cam_object
        if robot_T_base_gripper is None:
            raise ValueError(
                f"cam{cam_id} 은 고정 카메라가 아니다. 그리퍼 카메라면 "
                f"robot_T_base_gripper 를 넘겨라. 쓸 수 있는 고정 카메라: "
                f"{sorted(self.T_base_cam)}")
        return (np.asarray(robot_T_base_gripper, float)
                @ self.T_gripper_cam @ T_cam_object)

    def to_base(self, cam_id: int, T_cam_object, robot_T_base_gripper=None,
                apply_ridge: Optional[bool] = None) -> np.ndarray:
        """위치만 필요할 때. 미터 단위 3-vector."""
        p = self.to_base_pose(cam_id, T_cam_object, robot_T_base_gripper)[:3, 3]
        return self._corrected(p, apply_ridge)

    def fuse(self, observations: Mapping[int, np.ndarray],
             robot_T_base_gripper=None,
             apply_ridge: Optional[bool] = None) -> np.ndarray:
        """여러 카메라의 관측을 합친다. 축별 median — PnP flip 한 건이 평균을 100 mm 넘게
        끌고 가기 때문에 평균이 아니라 median 이다."""
        points = []
        for cam_id, T in observations.items():
            if T is None:
                continue
            points.append(self.to_base_pose(cam_id, T, robot_T_base_gripper)[:3, 3])
        if not points:
            raise ValueError("관측이 하나도 없다")
        return self._corrected(np.median(np.asarray(points, float), axis=0), apply_ridge)

    def in_ridge_region(self, p: Sequence[float]) -> bool:
        """보정 계수를 학습한 x·y 범위 안인가."""
        region = self.meta["ridge_correction"].get("valid_region_m")
        if not region:
            return True
        p = np.asarray(p, float).reshape(3)
        return (region["x"][0] <= p[0] <= region["x"][1]
                and region["y"][0] <= p[1] <= region["y"][1])

    def _corrected(self, p: np.ndarray, apply_ridge: Optional[bool]) -> np.ndarray:
        use = self.use_ridge if apply_ridge is None else bool(apply_ridge)
        if not use:
            return p
        if not self.in_ridge_region(p):
            # [1,x,y] 선형식이라 학습 범위 밖은 외삽이다. 조용히 틀리느니 안 건드린다.
            warnings.warn(
                f"예측 위치 {np.round(p * 1000, 1)} mm 가 ridge 보정 학습 범위 밖이라 "
                f"보정을 건너뛴다 (범위 x{self.meta['ridge_correction']['valid_region_m']['x']}, "
                f"y{self.meta['ridge_correction']['valid_region_m']['y']} m)",
                RuntimeWarning, stacklevel=3)
            return p
        return p + np.array([1.0, p[0], p[1]], float) @ self.W


if __name__ == "__main__":
    calib = Calibration()
    print("캘리브 출처:", calib.meta["source"])
    print("쓸 카메라:", calib.use_cameras, " / 쓰지 말 것:", calib.meta["do_not_use"])
    for name, value in calib.meta["accuracy_mm"].items():
        print(f"  {name:28s} {value}")
    for cam_id in sorted(calib.T_base_cam):
        t = calib.T_base_cam[cam_id][:3, 3] * 1000.0
        print(f"  T_base_C{cam_id} 위치(mm) = {np.round(t, 1)}")
