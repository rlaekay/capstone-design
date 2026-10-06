"""The single host clock every stream is stamped with.

monotonic_ns never jumps (NTP adjusts wall time, not this). A wall-clock anchor taken at
start-up converts monotonic stamps to unix time for reference only.
"""
import time

T0_NS = time.monotonic_ns()
UNIX0_NS = time.time_ns()


def now_ns() -> int:
    return time.monotonic_ns()


def rel_s(ns: int) -> float:
    """Seconds since process start; used on the live GUI timeline."""
    return (ns - T0_NS) / 1e9


def unix_s(ns: float) -> float:
    return (UNIX0_NS + (ns - T0_NS)) / 1e9
