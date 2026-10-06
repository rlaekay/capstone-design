# 자연스러운 속도 변화 논문형 10개

학습 구간 10 × 12초 = 120초. 각 파일의 준비/복귀 구간은 별도입니다.
원본 조건별 train/001.csv의 gripperframe 경로를 평활화하고 현재 작업공간에 위치/크기를 맞춘 뒤,
공간상의 추가 XY 왕복 없이 단조 증가하는 진행률로 시간만 재매개화했습니다.
`source_displacement_scale`, `origin_world_m`, `timewarp_beta`는 config.json의 revised_blocks에 있습니다.
지정한 기하 경로에 대한 보간/IK 오차 최대 약 2.27 µm, 추가 XYZ 진동은 0입니다.
시뮬레이션의 물리 추종 오차는 이 IK 오차와 다르며 physics.json에 별도로 기록되어 있습니다.
열린 그리퍼 1.2 rad, 드론/책상/양쪽 클램프 포함 모델로 모든 기록을 검증했습니다.

프로젝트 루트에서:
```bash
MUJOCO_GL=glfw .venv/bin/python scripts/view_library.py --library outputs/paper_timewarp_10 --auto
```
12초 구간만 보려면 --training-only 추가.
원 논문 기록 그대로의 재생은 이 세트가 아니라 paper_original_10입니다.
