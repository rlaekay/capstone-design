#!/bin/bash
# NA logger 실행: ./run.sh [--teleop] [--sim] [--no-cf] [--config config.toml] [--port 8080]
cd "$(dirname "$0")"
exec .venv/bin/python -m nalog "$@"
