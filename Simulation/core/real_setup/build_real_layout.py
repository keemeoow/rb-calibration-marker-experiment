#!/usr/bin/env python3
"""실제 촬영 데이터에서 시뮬레이터가 쓸 배치(참값)를 뽑아낸다.

뽑는 것
  bTf   고정 카메라 자세 (베이스 기준)      <- Step3 캘리브 결과
  gTc   그리퍼 - 그리퍼 카메라 hand-eye     <- Step3 캘리브 결과
  bTo   세트별 큐브 자세                    <- Step3 T_base_O_by_set.json
  bTg   이벤트별 그리퍼 자세                <- meta.json robot_pose_matrix_4x4
  event_set  이벤트 -> 세트 매핑

이 배치를 참값으로 심어두면, 시뮬레이션은 실제와 같은 기하에서
FK 오차나 카메라 노이즈만 바꿔가며 실험할 수 있다.

실행:
  python3 build_real_layout.py \
      --meta   ../../../data/session02/calib_train/meta.json \
      --calib  ../../../data/session01/calib_out_v2 \
      --out    real_layout_session02.npz
"""
import os
import json
import glob
import argparse

import numpy as np


def _rpy_xyz_to_T(vals):
    """capture_cube_center_6dof -> 4x4. 위치는 mm, 각도는 도(roll,pitch,yaw)."""
    x, y, z, rx, ry, rz = [float(v) for v in vals[:6]]
    t = np.array([x, y, z]) / 1000.0            # mm -> m
    cr, sr = np.cos(np.deg2rad(rx)), np.sin(np.deg2rad(rx))
    cp, sp = np.cos(np.deg2rad(ry)), np.sin(np.deg2rad(ry))
    cy, sy = np.cos(np.deg2rad(rz)), np.sin(np.deg2rad(rz))
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
    Ry = np.array([[cp, 0, sp], [0, 1.0, 0], [-sp, 0, cp]])
    Rx = np.array([[1.0, 0, 0], [0, cr, -sr], [0, sr, cr]])
    T = np.eye(4)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = t
    return T


def _mean_pose(Ts):
    """여러 4x4 의 평균. 회전은 SVD 로 SO(3) 에 투영."""
    Ts = np.asarray(Ts)
    out = np.eye(4)
    out[:3, 3] = Ts[:, :3, 3].mean(axis=0)
    U, _, Vt = np.linalg.svd(Ts[:, :3, :3].mean(axis=0))
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    out[:3, :3] = R
    return out


def build(meta_path, calib_dir, out_path):
    meta = json.load(open(meta_path))
    caps = meta["captures"]

    # ── 고정 카메라와 hand-eye ───────────────────────────────
    #   그리퍼 카메라 인덱스는 제외하고 고정 카메라만 모은다.
    grip_idx = int(meta.get("gripper_cam_idx", 2))
    bTf, cam_ids = [], []
    for f in sorted(glob.glob(os.path.join(calib_dir, "T_base_C*.npy"))):
        ci = int(os.path.basename(f).split("T_base_C")[1].split(".")[0])
        if ci == grip_idx:
            continue
        bTf.append(np.load(f))
        cam_ids.append(ci)
    gTc = np.load(os.path.join(calib_dir, "T_gripper_cam.npy"))

    # ── 세트별 큐브 자세 ─────────────────────────────────────
    #   Step3 가 vision 으로 구한 세트별 자세(T_base_O_by_set)를 쓴다.
    #   meta 의 capture_cube_center_6dof 는 로봇이 놓을 때의 지령값이라
    #   실제 관측과 어긋난다.
    byset_path = os.path.join(calib_dir, "internal_runtime", "T_base_O_by_set.json")
    if os.path.exists(byset_path):
        raw = json.load(open(byset_path))
        sets = sorted(int(k) for k in raw)
        bTo = np.stack([np.asarray(raw[str(s)]["transform"], dtype=np.float64)
                        for s in sets])
        cube_source = "step3_T_base_O_by_set"
    else:
        by_set = {}
        for c in caps:
            s, v = c.get("set_index"), c.get("capture_cube_center_6dof")
            if s is None or not v:
                continue
            by_set.setdefault(int(s), []).append(_rpy_xyz_to_T(v))
        sets = sorted(by_set)
        bTo = np.stack([_mean_pose(by_set[s]) for s in sets])
        cube_source = "meta_capture_cube_center_6dof"
    print(f"큐브 자세 출처: {cube_source}")

    # ── 이벤트별 그리퍼 자세 ─────────────────────────────────
    bTg, event_set = [], []
    for c in caps:
        s, M = c.get("set_index"), c.get("robot_pose_matrix_4x4")
        if s is None or not M:
            continue
        bTg.append(np.asarray(M, dtype=np.float64))
        event_set.append(int(s))
    bTg = np.stack(bTg)
    event_set = np.asarray(event_set, dtype=int)

    np.savez(out_path,
             bTf=np.stack(bTf), cam_ids=np.asarray(cam_ids, dtype=int),
             gTc=gTc, bTo=bTo, sets=np.asarray(sets, dtype=int),
             bTg=bTg, event_set=event_set,
             source_meta=os.path.abspath(meta_path),
             source_calib=os.path.abspath(calib_dir))

    print(f"고정 카메라 {len(bTf)}대 (인덱스 {cam_ids}, 그리퍼 카메라 {grip_idx} 제외)")
    print(f"세트 {len(sets)}개, 이벤트 {len(bTg)}개")
    print(f"큐브 중심 범위(m): x {bTo[:, 0, 3].min():.3f}~{bTo[:, 0, 3].max():.3f} | "
          f"y {bTo[:, 1, 3].min():.3f}~{bTo[:, 1, 3].max():.3f} | "
          f"z {bTo[:, 2, 3].min():.3f}~{bTo[:, 2, 3].max():.3f}")
    print(f"저장: {out_path}")
    return out_path


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.abspath(os.path.join(here, "..", "..", ".."))
    ap = argparse.ArgumentParser()
    ap.add_argument("--meta", default=os.path.join(
        root, "data/session02/calib_train/meta.json"))
    ap.add_argument("--calib", default=os.path.join(
        root, "data/session01/calib_out_v2"))
    ap.add_argument("--out", default=os.path.join(
        here, "real_layout_session02.npz"))
    a = ap.parse_args()
    build(a.meta, a.calib, a.out)
