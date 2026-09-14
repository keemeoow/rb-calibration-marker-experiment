# 촬영 데이터 인덱스

촬영 데이터 폴더 이름에는 **예외 없이 촬영 날짜 `_MMDD` 가 붙는다.** 같은 구성을
다른 날 다시 찍어도 이름만으로 구분되고, 덮어쓸 일이 없다.

- 촬영은 **그 촬영을 하는 루트의 `data/`** 안에 `session<NN>_<설명>_<MMDD>/` 로 쌓인다.
  `NN` 은 그 루트 안에서 순차로만 올라가고 재사용하지 않는다.
  - `03_capture.py` (composite rig) -> `zeus_gello_calibration/data/`
  - UR3 리그 스크립트 -> `ur3_calibration/data/`
- 최상위 `data/` 는 파이프라인이 바로 읽는 데이터셋(아카이브본·변환본)을 둔다.

폴더는 손으로 만들지 말고 촬영 스크립트가 할당하게 한다.

```bash
python 03_capture.py --data_root zeus_gello_calibration/data \
                     --session_label "zeus composite rig" ...
#   -> zeus_gello_calibration/data/session04_zeus_composite_rig_0914/   (다음 순번)

python ur3_calibration/capture_poses.py --session 1 ...
#   -> ur3_calibration/data/session1_handheld_fixed_cam_0914/
```

규칙의 원본은 `capture_pipeline/paths.py` 한 곳이고, 결과 폴더 규칙은
`calibration_pipeline/result_paths.py` (`ABLATION_TEST_result_<MMDD>/<세션명>/`) 에 있다.

## data/ — 파이프라인 데이터셋

| 세션 | 촬영일 | 내용 | 파일 | 용량 | 이전 위치 |
|---|---|---|---:|---:|---|
| `session01_calb_0626` | 2026-06-26 | 초기 ChArUco/큐브 촬영 + 당시 intrinsics 동봉 | 1063 | 76M | `calb_data_0626/` |
| `session02_NOUSE_session04_0814` | 2026-08-14 | 4캠 본 실험 촬영 — ABLATION_TEST table1 의 근거. NOUSE = 개선 프로토콜로 대체됨 | 429 | 239M | `data/session04/` |
| `session09_ur3_wrist_meta_0909` | 2026-09-09 | UR3 session3 을 calib_train/meta.json 형식으로 변환 (파생) | 61 | 71M | `data/ur3_session3/` |
| `session10_ur3_handheld_floor_meta_0909` | 2026-09-09 | UR3 session1+2 를 병합·변환 (파생) | 109 | 129M | `data/ur3_session12/` |

`session09`/`session10` 은 새 촬영이 아니라 아래 UR3 세션을 파이프라인이 읽는
`calib_train/meta.json` 형식으로 변환한 것이다 (`ur3_calibration/convert_to_meta.py`).
번호 03~08 이 비어 있는 것은 정상이다 — 세션 번호는 재사용하지 않는다.

## 리그별 data/ — 실제 촬영이 쌓이는 곳

| 경로 | 촬영일 | 내용 | 파일 | 용량 |
|---|---|---|---:|---:|
| `ur3_calibration/data/session1_handheld_fixed_cam_0909` | 2026-09-09 | UR3 · 보드를 손에 들고 고정 카메라 | 136 | 90M |
| `ur3_calibration/data/session2_floor_board_dual_cam_0909` | 2026-09-09 | UR3 · 바닥 보드 + 카메라 2대 | 109 | 73M |
| `ur3_calibration/data/session3_wrist_motion_gripper_cam_0909` | 2026-09-09 | UR3 · 손목 자세 변화 + 그리퍼 카메라 | 136 | 91M |
| `zeus_gello_calibration/data/session1_handheld_fixed_cam_0909` | 2026-09-09 | Zeus/GELLO · 보드를 손에 들고 고정 카메라 (capture_replayed 포함) | 288 | 182M |
| `zeus_gello_calibration/data/session2_floor_board_dual_cam_0909` | 2026-09-09 | Zeus/GELLO · 바닥 보드 + 카메라 2대 | 270 | 173M |
| `zeus_gello_calibration/data/session3_wrist_motion_gripper_cam_0909` | 2026-09-09 | Zeus/GELLO · 손목 자세 변화 + 그리퍼 카메라 | 270 | 174M |
