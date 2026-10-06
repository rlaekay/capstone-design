"""Crazyflie 연결 테스트: 라디오 스캔 → 연결 → 배터리 전압 출력 (프로펠러 안 돌림)."""
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

cflib.crtp.init_drivers()

# 이 기체의 라디오 주소는 기본값(E7E7E7E7E7)이 아니라 E7E7E7E7E8
DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E8"

uri = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_URI
if uri == "scan":
    found = cflib.crtp.scan_interfaces()
    print("발견된 인터페이스:", found)
    if not found:
        sys.exit("Crazyflie를 찾지 못했습니다. Crazyradio 연결 / 드론 전원을 확인하세요.")
    uri = found[0][0]

print("연결 중:", uri)
with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
    print("연결 성공!")
    log = LogConfig(name="Battery", period_in_ms=500)
    log.add_variable("pm.vbat", "float")
    with SyncLogger(scf, log) as logger:
        for i, (ts, data, _) in enumerate(logger):
            print(f"배터리 전압: {data['pm.vbat']:.2f} V")
            if i >= 4:
                break
    time.sleep(0.2)
print("완료")
