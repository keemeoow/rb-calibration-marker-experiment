# Zeus GELLO 캘리브레이션 데이터셋 수집 + 외부 GT 검증 실행 가이드

## 0. 사전 준비

- 큐브를 테이블 중앙에서 치우기 (session1 재파지 전 그리퍼가 지나갈 자리 확보)
- GELLO 텔레옵 종료 상태 확인 (아래 자동 이동 스크립트들은 motion ownership을
  직접 가져가야 해서, GELLO 텔레옵(`jello/gello_zeus_real_teleop.py`)이 켜져
  있으면 안 됨)

## 촬영 스크립트 2종: `capture_session.py` vs `replay_and_recapture.py`

데이터를 "찍는" 방법은 두 가지고, 서로 대체재가 아니라 순서대로 이어서 쓴다.

### `capture_session.py` -- GELLO로 직접 움직이면서 촬영 (최초 수집)

```bash
# 터미널 1: GELLO 텔레옵으로 로봇 조종 상태 켜두기
python jello/gello_zeus_real_teleop.py

# 터미널 2: GELLO로 원하는 자세로 옮긴 뒤 SPACE로 촬영
python zeus_gello_calibration/capture_session.py --session 1
python zeus_gello_calibration/capture_session.py --session 2 --num-poses 15
python zeus_gello_calibration/capture_session.py --session 3
```

- 로봇 움직임은 **사람(GELLO)**이 함 -- 이 스크립트는 로봇 서버에 읽기 전용으로
  붙어서 `get_state()`만 폴링, 움직임 명령은 전혀 안 보냄.
- SPACE를 누른 순간의 pose/joints(`robot.json`) + 카메라 4대 사진을
  `data/session{N}_.../capture/<idx:03d>/`에 저장.
- 새로운 자세를 자유롭게 만들어낼 수 있는 유일한 단계 (세 세션 다 최초엔 이걸로 찍음).
- `--num-poses`(기본 15)만큼 반복, `q`/ESC로 조기 종료, `--reset`으로 처음부터.
- **한계**: SPACE 누르는 순간에도 사람 손이 미세하게 움직이고 있을 수 있어서,
  기록된 pose와 실제 촬영된 이미지 사이에 아주 약간의 시간차(sync error)가
  생길 수 있음.

### `replay_and_recapture.py` -- 저장된 joints로 재생하며 재촬영 (정합성 확보)

```bash
python zeus_gello_calibration/replay_and_recapture.py --session 1                      # dry-run: 계획만 출력
python zeus_gello_calibration/replay_and_recapture.py --session 1 --execute            # 스텝별 확인하며 실행
python zeus_gello_calibration/replay_and_recapture.py --session 1 --execute --no-step  # 검증 후 연속 실행
python zeus_gello_calibration/replay_and_recapture.py --session 1 --execute --overwrite  # 원본 capture/ 에 덮어쓰기
python zeus_gello_calibration/replay_and_recapture.py --session 1 --execute --regrasp-joints  # 재파지부터 (기본 자세)
```

- `capture_session.py`가 이미 저장해둔 `robot.json`의 joint 값을 그대로 순서대로
  다시 불러서, **movej로 로봇을 그 자세까지 자동 이동시키고 완전히 멈춘 뒤** 촬영.
- 새 자세를 만드는 게 아니라 **기존 자세를 그대로 재생**하는 것 -- 사람 손 개입이
  없어서 "물리적으로 완전히 정지한 상태"에서 찍기 때문에 pose-이미지가
  정확히 같은 순간을 가리킴 (sync error 제거).
- 결과는 기본적으로 `capture_replayed/`에 별도 저장(원본 `capture/`는 안 건드림).
- `--regrasp-joints`(값 없이 주면 `REGRASP_JOINTS_DEFAULT`)를 주면, 재생 전에
  그리퍼를 열고 지정 자세로 이동 → 사용자가 큐브를 쥐여줌 → 그리퍼 닫기 절차를
  먼저 수행 (session1처럼 그리퍼로 물건을 쥔 채 촬영해야 하는 세션에 필요).
- **실제로 로봇을 자동으로 움직이는 스크립트다** -- 반드시 로봇 옆에서
  비상정지에 손 닿는 상태로, 처음엔 `--no-step` 없이 스텝별 확인하며 진행할 것.

| | `capture_session.py` | `replay_and_recapture.py` |
|---|---|---|
| 로봇 이동 주체 | 사람 (GELLO) | 스크립트 자동 (movej) |
| 용도 | 데이터 최초 수집 | 이미 찍은 데이터 재촬영(정합성 개선) |
| pose-이미지 정합 | 약간 어긋날 수 있음 | 완전 정지 후 촬영, 정확함 |
| 새 자세 생성 | O | X (저장된 자세 그대로 재생) |

session2는 x,y,rz만 쓰고 애초에 사람이 아니라 `session2_pick_and_place.py`
스크립트가 움직이므로(=sync 문제 자체가 없음), 재촬영 없이 바로 써도 된다.

## 1. session1 재촬영 (그리퍼로 큐브 쥔 채 고정캠 촬영)

```bash
python zeus_gello_calibration/replay_and_recapture.py --session 1 --execute --no-step --regrasp-joints
```

`--regrasp-joints`(값 없이)는 `REGRASP_JOINTS_DEFAULT[1]`(재파지 기본 자세)로
이동 후 그리퍼를 열고 사용자가 큐브를 쥐여주게 한 다음 닫고, 이어서 저장된
16개 자세를 순서대로 재생하며 촬영한다.

## 2. session2 (바닥 15곳에 pick-and-place 하며 촬영) (이 때 촬영된 값은 x, y, rz 만 사용하기 때문에 정확한 정렬과 높이 맞출 필요 없음)

```bash
python zeus_gello_calibration/session2_pick_and_place.py --execute --no-step \
  --move-speed 60 --descend-speed 30 --jnt-speed 20
```

## 3. session3 재촬영 (손목만 움직이며 바닥 마커보드, 그리퍼캠)

```bash
python zeus_gello_calibration/replay_and_recapture.py --session 3 --execute --no-step
```

## 4. 캘리브레이션 fit

```bash
python zeus_gello_calibration/fit_grasp_offset.py --capture-subdir capture_replayed \
  --out zeus_gello_calibration/pass1_grasp_offset_replayed.json

python zeus_gello_calibration/fit_calibration_methods.py       # px 학습, 3개 fit_*.json
python zeus_gello_calibration/fit_calibration_methods_mm.py    # mm 학습, 3개 fit_*_mm.json
```

## 5. 캘리브레이션 끝나고 외부 GT 실험

한 트라이얼당 아래 6단계를 반복한다 (GT 큐브를 매번 새 위치에 놓고).

1. **큐브를 테이블에서 치운 채로** 아래 joints로 이동 (준비 자세):
   ```
   27.14, -34.63, -96.67, -180.00, 48.71, 3.14
   ```
2. 그리퍼 **open/close**를 반복하며 GT 큐브 중점에 손가락이 정확히 맞도록 수동으로 정렬한다.
3. 그리퍼 **open**, 큐브 위로 **5cm 이동** — 이때의 **joint 좌표를 기록**해둔다 (이후 트라이얼마다 재사용할 "picking 기준 자세").
4. 그리퍼캠까지 위로 올라가서 촬영할 자세로 movej:
   ```
   42.97, 0.12, -105.91, -14.90, -59.33, -104.72
   ```
5. 촬영 + 6가지 방식(통합/독립 × no-fk/raw-fk × px/mm) 결과 비교:
   ```bash
   python zeus_gello_calibration/gt_compare_fits.py
   ```
6. **다음 트라이얼 준비**: step 3에서 기록한 joint 좌표로 이동 → 5cm 내려가서 그리퍼 **close**(큐브 집기) → 5cm 올라오기 → 다음 촬영 위치로 이동 → 5cm 내려가서 큐브 내려놓기 → **step 3부터 반복**.

## 안전 수칙

`replay_and_recapture.py`, `session2_pick_and_place.py`, `gt_pick_test.py --execute`
등 실제로 로봇을 자동으로 움직이는 스크립트는 전부, 처음 실행 시 로봇 옆에서
비상정지에 손이 닿는 상태로 `--no-step` 없이 스텝별로 확인하며 저속으로
진행할 것.
