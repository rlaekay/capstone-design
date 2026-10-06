#!/bin/bash
# MuJoCo 디지털 트윈 (로거가 먼저 실행 중이어야 함): ./twin.sh [--url http://<로거 IP>:8080]
cd "$(dirname "$0")"
exec .venv/bin/mjpython -m nalog.twin "$@"
