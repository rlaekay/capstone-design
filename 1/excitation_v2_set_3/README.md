# excitation_v2_set_3

Simulation-only motion bank. Gripper reference = 1.2 rad.

10 training clips × 12 seconds = 120 seconds. Each full replay also contains a collision-checked home approach and return.

Files in each motion_000 ... motion_009 folder:
- training_trajectory.npz / training_reference.csv: the 12-second command, local time starts at zero.
- training_imu.npz / training_imu.csv: synchronized simulated telemetry for those 12 seconds, local time starts at zero.
- trajectory.npz / reference.csv: complete home-to-home command including approach and return.
- imu.npz / imu.csv: complete physics telemetry; ideal simulated IMU, not real sensor data.
- end_effector.csv: full-replay gripperframe XYZ in world coordinates.
- config.json: training_window_s identifies the crop in full-replay time; source provenance and generation parameters.
- physics.json / summary.json: validation and simulation results.

Run from /home/robocup-home/kjt/force_modeling:
```bash
MUJOCO_GL=glfw .venv/bin/python scripts/view_library.py --library outputs/excitation_v2_set_3 --index 0 --auto
```
Add --training-only for just the 12-second clip. N/P or arrow keys: next/previous; Space: pause; R: restart; C: collision geometry.

Payload condition names in paper_xy_10 identify the source CSV only. No grasped object geometry/mass or force labels are included.
Collision validation applies to the current provisional robot/desk/clamp/drone model; it is not a hardware safety certification.
