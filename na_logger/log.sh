#!/bin/bash
# 페이로드 데이터 수집:  ./log.sh
#   1) 로거가 꺼져 있으면 켬 (켜져 있으면 그대로 사용)
#   2) 팔로워 + Crazyflie 연결 확인
#   3) 무게(g) 입력 → 그리퍼 토크 풀림 → 물체를 테이프로 고정하고 Enter → 30개 궤적 자동 재생·저장
#      (그리퍼는 그 자리에 고정, 추가 조임 없음)
#      (paper_timewarp_10, excitation_v2_set_1, excitation_v2_set_2)
#   4) 끝나면 그리퍼 토크를 다시 풀어줌 → 다음 무게
# 동작 중 Ctrl+C: 동작 정지 + 전체 토크 풀림 (팔이 처질 수 있음), 방금 데이터를 지울지 물어봄
# 데이터: data/loaded/payload_<g>g/<회차>_<세트>_<궤적>/ (meta.json, training.csv, full.csv, raw/)
set -u
cd "$(dirname "$0")"
URL="http://localhost:8080"
mkdir -p logs

if curl -sf "$URL/api/status" >/dev/null 2>&1; then
  echo "● 로거 실행 중 ($URL)"
else
  if pgrep -f "cfclient" >/dev/null; then
    echo "⚠ cfclient가 켜져 있으면 Crazyradio를 쓸 수 없어요. cfclient를 먼저 종료하세요."
  fi
  LOG="logs/logger-$(date +%Y%m%d-%H%M%S).log"
  nohup .venv/bin/python -m nalog --port 8080 >"$LOG" 2>&1 &
  echo "● 로거 시작 (로그: $LOG) — 이 창을 닫아도 로거는 계속 돌아요"
  for _ in $(seq 1 40); do
    curl -sf "$URL/api/status" >/dev/null 2>&1 && break
    sleep 0.5
  done
fi

exec .venv/bin/python -m nalog.campaign
