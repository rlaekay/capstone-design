#!/bin/bash
# NA logger 한 번에 켜기 — 드론 전원을 넣고 팔(USB + 12 V)을 연결한 뒤:  ./start.sh
#   1) 로거(GUI 서버, 포트 8080) 실행 — 이미 켜져 있으면 그대로 사용
#   2) 팔로워 SO-101과 Crazyflie 연결 확인 (최대 40초 기다림)
#   3) 브라우저로 GUI 열기 + MuJoCo 디지털 트윈 창 띄우기
# 끝내기: 이 터미널에서 Ctrl+C (트윈 창만 닫으면 로거와 GUI는 계속 돌아감)
# 옵션은 로거에 그대로 전달:  ./start.sh --sim   ./start.sh --no-cf
set -u
cd "$(dirname "$0")"
PORT=8080
URL="http://localhost:$PORT"
mkdir -p logs
LOGGER_PID=""
TWIN_PID=""

cleanup() {
  echo
  [ -n "$TWIN_PID" ] && kill "$TWIN_PID" 2>/dev/null
  if [ -n "$LOGGER_PID" ]; then
    echo "로거 종료 중 (진행 중인 회차는 저장하고 끝냄)…"
    kill -TERM "$LOGGER_PID" 2>/dev/null   # & jobs in a script ignore SIGINT
    wait "$LOGGER_PID" 2>/dev/null
  fi
  exit 0
}
trap cleanup INT TERM

if curl -sf "$URL/api/status" >/dev/null 2>&1; then
  echo "● 로거가 이미 실행 중 → 그대로 사용합니다 ($URL)"
else
  if pgrep -f "cfclient" >/dev/null; then
    echo "⚠ cfclient가 켜져 있으면 Crazyradio를 쓸 수 없어요. cfclient를 먼저 종료하세요."
  fi
  LOG="logs/logger-$(date +%Y%m%d-%H%M%S).log"
  .venv/bin/python -m nalog --port "$PORT" "$@" >"$LOG" 2>&1 &
  LOGGER_PID=$!
  echo "● 로거 시작 (로그: $LOG)"
  for _ in $(seq 1 40); do
    curl -sf "$URL/api/status" >/dev/null 2>&1 && break
    if ! kill -0 "$LOGGER_PID" 2>/dev/null; then
      echo "✗ 로거가 시작하지 못했어요. 로그 마지막 부분:"; tail -20 "$LOG"; exit 1
    fi
    sleep 0.5
  done
fi

# ---- device check: follower + Crazyflie
.venv/bin/python - "$URL" <<'EOF'
import json, sys, time, urllib.request
url = sys.argv[1]
t0, last = time.time(), ""
while True:
    try:
        st = json.load(urllib.request.urlopen(url + "/api/status", timeout=2))
    except Exception as e:  # noqa: BLE001
        st = None
    if st:
        arm, cf = st["arm"], st["cf"]
        f = arm.get("follower", {})
        arm_ok = f.get("connected") and arm.get("rate_hz", 0) > 50
        cf_ok = cf.get("state") == "connected" and cf.get("rate_hz", 0) > 50
        a = (f"팔로워 ✓ {f.get('port')}  {f.get('voltage') or 0:.1f} V  {arm.get('rate_hz', 0):.0f} Hz"
             if arm_ok else f"팔로워 … {arm.get('message', '')} {list(arm.get('port_notes', {}).values())[:1]}")
        c = (f"드론   ✓ {cf.get('uri')}  {cf.get('rate_hz', 0):.0f} Hz  배터리 {cf.get('battery_v') or 0:.2f} V"
             if cf_ok else f"드론   … {cf.get('state')} {cf.get('message', '')}")
        line = f"  {a}\n  {c}"
        if line != last:
            print(line, flush=True)
            last = line
        if arm_ok and cf_ok:
            print("● 팔과 드론 모두 연결됨")
            break
    if time.time() - t0 > 40:
        print("⚠ 40초 안에 모두 연결되지 않았어요. GUI와 트윈은 띄우고, 연결되면 자동으로 들어옵니다.")
        break
    time.sleep(1)
EOF

# ---- GUI + twin
open "$URL"
.venv/bin/mjpython -m nalog.twin --url "$URL" >logs/twin.log 2>&1 &
TWIN_PID=$!
echo "● GUI: $URL  (같은 Wi-Fi: http://$(scutil --get LocalHostName 2>/dev/null).local:$PORT)"
echo "● MuJoCo 트윈 창을 띄웠어요. 끝내려면 이 터미널에서 Ctrl+C"

if [ -n "$LOGGER_PID" ]; then
  wait "$LOGGER_PID"
else
  wait "$TWIN_PID"
fi
