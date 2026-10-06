# Crazyflie 개발 환경 (macOS, Apple Silicon)

설치된 것
- Homebrew: `uv`, `libusb` (Crazyradio USB 통신용)
- `.venv` (Python 3.12): `cfclient 2026.8`, `cflib 0.1.33`, `PyQt6`

## 실행
```bash
cd ~/my/Artemis/crazyflie
.venv/bin/cfclient                 # GUI 클라이언트
.venv/bin/python test_connect.py   # 연결 → 배터리 전압 확인 (인자로 다른 URI 또는 scan)
```

## 연결 순서
1. Crazyradio 2.0/PA 동글을 맥북 USB에 꽂기 (USB-C 맥이면 허브/젠더 사용). macOS는 드라이버 설치 불필요.
2. Crazyflie 전원 켜기 (평평한 곳, 부팅 중 움직이지 않게).
3. cfclient → Address 칸에 `0xE7E7E7E7E8` 입력 → Scan → `radio://0/80/2M/E7E7E7E7E8` 선택 → Connect.
   - **이 기체 URI: `radio://0/80/2M/E7E7E7E7E8`** (주소가 기본값 E7E7E7E7E7이 아니라서 기본 스캔에 안 잡힘)
   - 확인 결과(2026-09-27): 라디오 연결 OK, 배터리 4.19V, IMU 100Hz 로깅 OK
4. 처음이면 Connect > Bootloader 에서 펌웨어 최신으로 업데이트 권장.

## 문제 해결
- "Cannot find a Crazyradio Dongle": 동글 인식 안 됨 → `system_profiler SPUSBDataType | grep -i crazy` 로 확인, 다른 포트/허브 시도.
- 스캔 결과 없음: 드론 전원/배터리, 채널·주소가 기본값에서 바뀌었는지 확인.
- USB 케이블로 직접 연결도 가능 (`usb://0`), 단 데이터 케이블이어야 함.
- Crazyradio 동글 펌웨어가 오래됐으면 https://www.bitcraze.io/documentation/repository/crazyradio2-firmware/ 참고.
