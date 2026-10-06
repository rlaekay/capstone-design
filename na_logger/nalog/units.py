"""STS3215 raw register units -> physical units."""
from __future__ import annotations

POS_DEG_PER_STEP = 360.0 / 4095   # same as LeRobot DEGREES mode (resolution - 1)
VEL_DPS_PER_STEP = 360.0 / 4096   # Present_Velocity is in steps/s
LOAD_PCT_PER_UNIT = 0.1           # Present_Load: 0..1000 = 0..100 % of max torque, signed
CUR_MA_PER_UNIT = 6.5             # Present_Current: 6.5 mA per unit, signed
VOLT_V_PER_UNIT = 0.1


def pos_deg(raw, mid):
    return None if raw is None or mid is None else (raw - mid) * POS_DEG_PER_STEP


def scaled(raw, k):
    return None if raw is None else raw * k
