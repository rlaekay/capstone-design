# NA Logger — SO-101 × Crazyflie 동기화 로거

NeuralActuator(RSS 2026) 방식의 데이터 수집용 프로그램이에요. 한 맥북에서 아래를 한꺼번에 처리해요.

- **SO-101 리더→팔로워 텔레옵**: LeRobot과 같은 방식으로 리더 관절값을 팔로워 범위에 맞춰 옮겨요.
- **팔로워 액추에이터 텔레메트리**: 명령(Goal), 위치, 속도, 부하(signed load), 전류, 전압, 온도
- **팔 끝 Crazyflie IMU**: 가속도, 자이로, 자세

모든 스트림은 **맥북 시계 하나**(`time.monotonic_ns`)로 시간이 찍혀요. 기록은 회차별로 나뉘고, 결과는 같은 Wi-Fi에서 접속하는 웹 화면으로 확인하고 다운로드해요.

```
리더 SO-101 ──USB──┐
팔로워 SO-101 ─USB─┼── 맥북: nalog ──── 웹 GUI (http://<맥 IP>:8080, 같은 Wi-Fi)
Crazyradio ───USB──┘      │
   ⇣ 2.4 GHz              └── data/T0001_.../merged.csv …
Crazyflie (팔 끝)
```

## 한 번에 켜기 (권장)

드론 전원을 켜고 팔(USB + 12 V)을 연결한 뒤:

```bash
cd ~/my/Artemis/na_logger && ./start.sh
```

로거를 켜고, 팔과 드론 연결을 확인한 뒤(최대 40초), GUI(브라우저)와 MuJoCo 트윈 창을 띄워요. 끝낼 때는 그 터미널에서 Ctrl+C를 누르세요. GUI 주소는 항상 같고(`http://Kims-MacBook-Pro.local:8080`), GUI 파일이 바뀌거나 로거가 다시 켜지면 열린 페이지가 알아서 새로고침돼요.

리더 팔과 텔레옵은 이 프로젝트에서 쓰지 않아요. 기본값이 `leader_port = "none"`이에요.

## 실행 (개별)

```bash
cd ~/my/Artemis/na_logger
./run.sh              # 실제 하드웨어
./run.sh --teleop     # 두 팔이 잡히면 텔레옵 자동 시작
./run.sh --sim        # 하드웨어 없이 시뮬레이션 (GUI·파이프라인 확인용)
./run.sh --no-cf      # Crazyflie 없이
```

실행하면 터미널에 접속 주소가 나와요. 같은 Wi-Fi의 다른 기기에서 그 주소로 접속하면 돼요.

- 처음 실행할 때 macOS가 "들어오는 연결 허용"을 물어보면 **허용**을 눌러야 다른 기기에서 접속할 수 있어요.
- 실행 중에는 `caffeinate`로 맥이 잠들지 않게 해 둬요. 화면만 꺼지고 기록은 계속돼요.
- **cfclient는 반드시 꺼 두세요.** Crazyradio 동글은 한 프로그램만 쓸 수 있어요.
- LeRobot 텔레옵(`lerobot-teleoperate` 등)도 꺼 두세요. 시리얼 포트는 한 프로그램만 쓸 수 있고, 이 로거가 텔레옵까지 직접 해요.

## 연결 순서

1. 리더와 팔로워의 USB, 팔로워 12 V 전원, Crazyradio를 연결하고 Crazyflie 전원을 켜요.
2. `./run.sh`를 실행하면 포트를 자동으로 찾아요.
   - 서보 전압이 8 V 이상이면 팔로워(STS3215 C018, 12 V), 그보다 낮으면 리더(5 V)로 판별해요.
   - 판별이 틀리면 `config.toml`에서 `follower_port`와 `leader_port`를 직접 지정하세요.
3. GUI의 **텔레옵 시작** 버튼을 누르거나 `--teleop`으로 실행해요.
   - 버튼은 로거가 실행 중인 맥에서만 보여요. 같은 Wi-Fi의 다른 사람은 로봇을 움직일 수 없어요.
   - 시작하면 리더 토크를 끄고, 팔로워 목표를 현재 위치로 맞춘 뒤 팔로워 토크를 켜요.
   - 처음 5초 동안은 목표값 변화를 최대 60°/s로 제한해요(소프트 스타트). 리더와 자세가 달라도 팔로워가 급하게 튀지 않아요.
4. 리더를 움직이면 GUI가 대기 화면에서 **자동으로 깨어나고**, 자동 회차가 켜져 있으면 기록이 시작돼요.
   - 움직임이 3초 멈추면 그 회차가 끝나요.
   - 움직임 1초 전(pre-roll)부터 저장돼요.

텔레옵 매핑은 LeRobot 기본값(RANGE_M100_100 정규화)과 같아요. 범위는 LeRobot 캘리브레이션 때 서보 EEPROM에 저장된 Min/Max_Position_Limit를 읽어서 써요. 그래서 이 맥에 LeRobot 캘리브레이션 파일이 없어도 돼요. 단, 두 팔 모두 LeRobot으로 한 번은 캘리브레이션되어 있어야 해요.

## GUI

- **상단**: 팔로워, 리더, Crazyflie 연결 상태와 수신 Hz, 시간 동기 품질, 자동 회차 스위치, 수동 기록 버튼
- **그래프**: 전체 관절 위치 / 선택 관절의 리더 목표·명령·실제 / 속도·부하 / IMU 자이로·가속도
- **관절 상태 표**: 온도가 55 °C 이상이면 주황, 65 °C 이상이면 빨강으로 표시돼요.
- **회차 목록**
  - 메모: 예를 들어 "payload 200g"
  - **CSV 다운로드**: merged.csv 하나
  - **원본 zip**: 원시 CSV 전부와 meta.json
  - **그래프로 보기**: 끝난 회차 전체를 다시 봐요.
  - 맨 위 **전체 CSV (zip)**: 모든 회차의 merged.csv를 한 번에 받아요.
- **대기 화면**: 움직임 없이 60초가 지나면 대기 화면으로 바뀌고, 로봇이 움직이면 자동으로 꺼져요.

## 데이터 (`data/T0001_20260930-142233/`)

| 파일 | 내용 |
|---|---|
| `merged.csv` | **분석용.** 팔로워 샘플마다 한 행이에요. 모든 값이 물리 단위이고, IMU는 같은 시각으로 보간돼 있어요. |
| `arm.csv` | 팔로워와 리더의 원시 레지스터 값이에요. 무손실이고 샘플마다 요청·응답 시각이 있어요. |
| `imu.csv`, `att.csv` | Crazyflie 원시 로그예요. 맥 수신 시각과 Crazyflie 자체 타임스탬프가 같이 있어요. |
| `meta.json` | 회차 정보가 들어 있어요. 캘리브레이션, 서보 EEPROM 설정(P/I/D, 토크 한계 등), 설정값, 시계 피팅 결과, IMU 지연 추정치예요. |

`merged.csv` 주요 열은 아래와 같아요. 관절 이름은 `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`예요.

- `t_s`: 회차 시작 기준 초, `t_host_ns`: 맥 monotonic 시각, `unix_s`: 참고용 벽시계
- `teleop`: 0 꺼짐 / 1 동작 중 / 2 자세 유지
- `{관절}_pos_deg`: 실제 위치, `{관절}_cmd_deg`: 서보 Goal_Position(q_cmd), `{관절}_target_deg`: 리더를 팔로워 좌표로 옮긴 목표
- `{관절}_vel_dps`, `{관절}_load_pct`(부호 있음, 논문의 SO-101 effort 입력), `{관절}_cur_mA`, `{관절}_volt_V`, `{관절}_temp_C`, `{관절}_torque`, `{관절}_pos_raw`, `{관절}_cmd_raw`
- `imu_acc_{x,y,z}_g`, `imu_gyro_{x,y,z}_dps`, `cf_{roll,pitch,yaw}_deg`
- `imu_age_ms`: 가장 가까운 IMU 샘플까지의 거리예요. 25 ms를 넘는 구간은 IMU 값이 빈칸이에요.

각도 기준은 LeRobot DEGREES 모드와 같아요: (raw − 범위 중앙) × 360/4095.

## 시간 동기화 방식

1. **SO-101**: 서보에는 시계가 없어요. 그래서 sync read 요청을 보낸 시각과 응답을 받은 시각의 **중간값**을 샘플 시각으로 써요. 한 번 읽는 데 보통 2~4 ms 걸려요.
2. **Crazyflie**: 로그 패킷마다 Crazyflie 자체 ms 타임스탬프가 있어요. `맥 수신시각 − CF시각`의 1초별 최솟값들(가장 빨리 도착한 패킷)에 직선을 맞춰 CF 시계를 맥 시계로 바꿔요. 이렇게 하면 라디오 지연의 들쭉날쭉함이 빠지고, 두 시계의 속도 차(drift)도 보정돼요. 회차가 끝나면 전체 데이터로 다시 피팅해요.
3. **남는 고정 지연**(센서 필터, 최소 전송 지연)은 회차마다 `|자이로|`와 관절 각속도 크기를 상호상관해서 추정해요. 결과는 GUI 회차 목록의 "IMU 지연"과 `meta.json`의 `imu_lag`에 있어요.
   - **보정 절차**: 손목(wrist_flex)이나 어깨 회전(shoulder_pan)을 1~2 Hz로 크게 여러 번 흔드는 회차를 따로 기록해요.
   - 상관계수(r)가 높게 나온 값을 `imu_latency_ms`에 넣으면, 이후 merged.csv에서 그만큼 IMU 시각을 앞당겨요.

시뮬레이션으로 확인한 결과예요. 조건은 센서 지연 25 ms, 최소 라디오 지연 2.5 ms, 수신 지터, 5% 재전송 버스트, 시계 drift 40 ppm이에요.
- 시계 피팅 잔차는 0.13~0.34 ms였어요.
- IMU 지연 추정은 회차별로 26.3~28.1 ms였어요. 참값은 27.5 ms예요.
- 추정은 `|자이로|² ≈ q̇ᵀ M q̇` 최소제곱 피팅을 써요. 기구학이나 관절 부호를 몰라도 되고, 평행한 피치 관절 3개가 합쳐지는 것도 정확히 반영돼요.


## IMU 정렬 자동 식별 (alignid)

드론을 손으로 붙여 정렬을 모를 때 쓰는 기능이에요. GUI의 **IMU 정렬 (alignid)** 카드에서 **정렬 시작**을 한 번 누르면 아래가 자동으로 진행돼요. 시작 버튼은 로거 Mac에서만 보여요.

1. **점검**: 현재 자세에서 시작하는 약 80~90초짜리 경로를 만들어요.
   - 관절 범위와 속도(220°/s 이하)를 확인해요.
   - MuJoCo 모델(`model/scene.xml`)로 충돌을 검사해요. 8 mm 이내로 가까워지면 충돌로 봐요.
   - 충돌하는 자세나 흔들기 구간은 반대 방향으로 바꾸거나 줄여서 자동으로 다시 계획해요.
   - 시작 자세가 관절 범위 밖(예: 접힌 휴식 자세)이면 시작하지 않아요. 토크가 꺼진 상태에서 팔을 손으로 중간 자세로 옮긴 뒤 다시 누르세요.
2. **동작 + 기록**: 정지 자세 9개를 각 2.2초씩 유지한 뒤, 손목 roll·flex, 팔꿈치, 어깨 lift·pan을 하나씩 흔들고 마지막에 섞어서 흔들어요. 이 과정이 `alignid` 회차로 기록돼요.
   - 목표값 변화는 300°/s로 제한돼요.
   - **중단** 버튼을 누르면 그 자리에서 자세를 유지해요.
3. **계산**: 아래 값을 차례로 구해요.
   - 시간지연
   - 자이로 bias (정지 구간에서 측정)
   - 피치 관절 0점 보정 (±15° 탐색)
   - 회전 `R_gi`: IMU 자이로와 FK로 계산한 그리퍼 각속도를 Kabsch 정렬
   - 가속도 bias와 레버암 `p_gi`: 비력(specific force) 최소제곱
   - 결과는 `data/calib/current.json`에 저장되고, 이후 새 회차에 자동으로 적용돼요.

시뮬레이션(실제 장착: 모델 대비 10.9° 회전, 위치 4~5 mm 차이, 피치 0점 +3°)으로 확인한 결과예요.
- 회전 오차: 0.01°
- 레버암 오차: 2 mm 이내
- 피치 0점, bias: 참값과 일치

## payload 가속도와 force label (merged.csv)

정렬 결과가 있으면 merged.csv에 아래 열이 추가돼요.

기준점은 **payload 무게중심**(모델의 `payload_com` 사이트, `gripperframe`(TCP)과 같은 점)이에요. label이 F = m(g − a_com)이라서 무게중심의 가속도가 필요해요. 드론 자체를 payload로 쓰면 config에서 `accel_point = "drone"`으로 바꾸세요.

| 열 | 내용 |
|---|---|
| `gyro_grip_{x,y,z}_rps` | 그리퍼 좌표계 각속도 |
| `acc_payload_grip_{x,y,z}_mps2` | payload 무게중심의 비력 (그리퍼 좌표계, 중력 포함) |
| `acc_payload_base_{x,y,z}_mps2` | payload 무게중심의 가속도 (base 좌표계, 중력 제거) |
| `acc_payload_fk_base_{x,y,z}_mps2` | 같은 값을 관절각만으로 계산한 것 (FK 2차 미분, 검증용) |
| `force_label_base_{x,y,z}_N` | GUI에 payload 질량을 넣었을 때: F = m(g − a), 환경→로봇 (정지 시 −mg ẑ) |
| `force_fk_base_{x,y,z}_N` | 같은 force를 FK 가속도로 계산한 비교값 |

변환식이에요. 강체 위의 두 점 사이 변환이라 회전 R 하나로는 부족하고, 매 순간의 ω, α가 필요해요.

`f_P = R_gi (f_imu − b_a) + α×r + ω×(ω×r)`, `r = p_TCP − p_IMU` (그리퍼 좌표계)

IMU에서 TCP까지는 9.5 cm예요. 시뮬레이션에서 회전만 적용하면 오차가 0.4~0.7 m/s² RMS(최대 2 m/s²)였고, 위 식을 쓰면 0.07~0.09 m/s²였어요.

## MuJoCo 디지털 트윈

```bash
./twin.sh                                  # 로거가 실행 중이어야 함 (같은 Mac)
./twin.sh --url http://<로거 IP>:8080        # 같은 Wi-Fi의 다른 Mac에서
```

- 모델 `model/torque_scene.xml`이 실제 관절각을 따라 움직여요. 드론은 alignid가 구한 위치에 표시돼요.
- **화살표** (중력 제거한 가속도, 2 cm = 1 m/s²): 빨강은 IMU 실측, 파랑은 같은 순간 관절 움직임으로 MuJoCo가 예측한 값(`mj_inverse`), 초록은 TCP 가속도(실측을 변환한 값)예요.
- **텍스트**: 관절각, 모델 관절 토크, 비력·자이로 실측과 모델 값
- **그래프**: 비력과 자이로 x/y/z, 실측(진한 선)과 모델(옅은 선)
- 확인용 스냅샷: `.venv/bin/python -m nalog.twin --snapshot twin.png`

## 파일 구조

```
nalog/
  sts.py       STS3215 레지스터 읽기/쓰기 (Feetech SDK, sign-magnitude 디코딩)
  arms.py      포트 탐색, 리더/팔로워 판별, 100 Hz 루프, 텔레옵
  imu.py       Crazyflie 연결/재연결, 로그 블록
  sync.py      시계 피팅, IMU 지연 추정, 움직임 감지
  recorder.py  라이브 상태, 회차 자동 분할, CSV 기록
  finalize.py  회차 후처리 → merged.csv, preview.json
  server.py    웹 서버 (WebSocket, 다운로드 API)
  sim.py       하드웨어 없는 시뮬레이션 (FK 기반 IMU, 숨겨진 참 장착값)
  kin.py       MuJoCo 모델 FK, 충돌 검사, 관절 매핑
  alignid.py   IMU 정렬 자동 식별 (경로 계획, 실행, 계산)
  accel.py     IMU → 그리퍼 점 가속도 변환
  twin.py      MuJoCo 디지털 트윈 뷰어
model/         사용자 MJCF (scene.xml, torque_scene.xml) + 메시
  static/      GUI
tests/         핵심 로직 테스트: .venv/bin/python -m pytest -q
```

## 문제 해결

- **팔로워가 안 잡힘**: `ls /dev/cu.usbmodem*`로 포트를 확인하세요. GUI 팔로워 표시에 마우스를 올리면 포트별 사유가 보여요. "서보 응답 없음"이면 12 V 전원을 확인하세요.
- **리더/팔로워가 바뀌어 잡힘**: `config.toml`에 `follower_port`와 `leader_port`를 지정하세요.
- **Crazyflie 연결 안 됨**: cfclient를 끄세요. 주소가 `radio://0/80/2M/E7E7E7E7E8`인지 확인하세요.
- **텔레옵 시작 불가: 캘리브레이션 범위 비정상**: LeRobot으로 두 팔을 캘리브레이션하세요(`lerobot-calibrate`).
