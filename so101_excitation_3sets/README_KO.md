# SO-ARM101 익사이테이션 3세트 이관 패키지

이 폴더 전체를 옮기세요. 원래 PC의 /home 경로나 .venv는 필요 없습니다.
Ubuntu/Linux, Python 3.10, MuJoCo 3.3.2에서 확인한 시뮬레이션 재생 패키지입니다.

## 구성과 분량

- outputs/excitation_v2_set_1: 10개 × 12초 = 120초
- outputs/excitation_v2_set_2: 10개 × 12초 = 120초
- outputs/excitation_v2_set_3: 10개 × 12초 = 120초
- 전체 학습 명령 구간: 30개, 360초(6분)
- 준비/복귀까지 포함한 전체 명령 길이: 세트별 209.16 / 211.36 / 208.04초
- 각 세트의 motion_000 ~ motion_009에 명령과 검증 결과가 있습니다.
- models, third_party: 책상, 양쪽 클램프, SO-ARM101, 고정된 드론 모델과 메시/라이선스
- planning, scripts: 재생/충돌 검사 코드
- catalog.csv: 30개 기록의 파일 경로, 학습 구간, 길이, 검사 결과
- CONTROL_HANDOFF_KO.md: 실제 제어 PC에 전달할 데이터 계약과 필요한 보정값

## 다른 PC에서 확인

압축을 푼 뒤 이 README가 있는 폴더에서 실행합니다.

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-viewer.txt
.venv/bin/python scripts/verify_package.py
MUJOCO_GL=glfw .venv/bin/python scripts/view_library.py --library outputs/excitation_v2_set_1 --auto
```

세트 2/3은 경로의 set_1을 set_2/set_3으로 바꾸세요.
전체 재생은 준비 이동 → 12초 학습 구간 → 복귀입니다.
`--training-only`를 붙이면 12초 학습 구간만 봅니다. 이 모드의 세트 전환은 시작 자세로 바로 이동하는 시각화이므로 실제 제어 순서로 사용하지 않습니다.
`--index 5`는 5번부터, `--speed 0.5`는 화면 재생을 절반 속도로 표시합니다.
N/P 또는 좌우 화살표: 다음/이전, Space: 일시정지, R: 재시작, C: 충돌 형상.
GUI가 없는 PC에서는 `--headless`로 모델/명령/기록의 호환성만 확인할 수 있습니다.
화면은 저장한 물리 시뮬레이션 상태를 재생하며 실제 로봇에 명령을 보내지 않습니다.

## 어떤 파일을 쓰나

| 파일 | 용도 |
|---|---|
| reference.csv | 준비/복귀 포함 전체 관절 명령. 실제 제어기로 옮길 때 우선 기준으로 삼는 경로 |
| training_reference.csv | 12초 학습 구간만 추출한 명령. 로봇이 이미 첫 자세에 도달한 뒤 사용 |
| trajectory.npz / training_trajectory.npz | 시간, 관절각, 각속도, 각가속도, 메타데이터를 포함한 원본 배열 |
| imu.csv / imu.npz | 전체 시뮬레이션의 실제 관절 상태와 이상적 IMU 기록 |
| training_imu.csv / training_imu.npz | 학습 구간의 시뮬레이션 기록. 시간은 0~12초 |
| end_effector.csv | 전체 재생 시간 기준 그리퍼 위치(world XYZ, m) |
| config.json | 관절 설정, 학습 구간의 전체 재생 시간 범위(training_window_s) |
| physics.json / summary.json | 물리 추종/충돌 검사 결과 |

CSV만 필요해도 reference.csv, training_reference.csv, config.json, catalog.csv와 제어 문서를 함께 전달하세요.
`imu.csv`의 실제 qpos를 목표 명령으로 혼동하지 마세요. 그것은 시뮬레이션에서 나타난 추종 결과입니다.
힘 라벨이나 실측 로봇/드론 데이터는 포함하지 않습니다.

## 모델과 검증 범위

원래 검증에 사용한 모델의 MJB를 함께 보관해 경로 변경에 따른 모델 해시 차이를 피했습니다.
재생은 models/bench/scene.mjb를 사용합니다. 상대 경로로 바꾼 scene.xml과 모든 메시도 들어 있습니다.
패키지 검증기는 해시, 원래 모델 해시, 상대경로 XML과 MJB의 수치 배열 일치, 30개 데이터의 시간/모델 일치를 확인합니다.
MuJoCo 버전을 임의로 바꾸거나 장착/환경 치수를 수정한 경우 기존 검증 결과를 그대로 사용하지 마세요.
충돌 검사는 기존 열린 그리퍼(1.2 rad), 드론, 책상/클램프 모델 기준입니다. 실제 파지 물체 형상과 측정되지 않은 장착 치수는 보장 범위에 없습니다.
