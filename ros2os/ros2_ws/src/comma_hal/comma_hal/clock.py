"""Conversions between the kernel clocks used by comma hardware and ROS time.

ROS 2 header stamps use the system (realtime) clock. The camera ISP reports
CLOCK_BOOTTIME, and the GPIO line-event ABI used for the IMU interrupt reports
CLOCK_REALTIME on the 4.9 AGNOS kernel but CLOCK_MONOTONIC on kernels >= 5.7.
"""
import time

CLOCKS = {
  "realtime": time.CLOCK_REALTIME,
  "monotonic": time.CLOCK_MONOTONIC,
  "boottime": getattr(time, "CLOCK_BOOTTIME", time.CLOCK_MONOTONIC),
}


def now_ns(clock: str) -> int:
  return time.clock_gettime_ns(CLOCKS[clock])


def offset_to_realtime_ns(clock: str) -> int:
  """Nanoseconds to add to a `clock` timestamp to express it in CLOCK_REALTIME."""
  if clock == "realtime":
    return 0
  # bracket the source clock read with two realtime reads to minimize skew
  r0 = time.clock_gettime_ns(time.CLOCK_REALTIME)
  c = time.clock_gettime_ns(CLOCKS[clock])
  r1 = time.clock_gettime_ns(time.CLOCK_REALTIME)
  return (r0 + r1) // 2 - c


def to_realtime_ns(ts: int, clock: str) -> int:
  return ts + offset_to_realtime_ns(clock)


def guess_clock(ts: int, candidates=("realtime", "monotonic", "boottime")) -> str:
  """Return the clock whose current value is closest to a fresh timestamp `ts`."""
  return min(candidates, key=lambda c: abs(now_ns(c) - ts))


def split_ns(ts: int) -> tuple[int, int]:
  """Split nanoseconds into (sec, nanosec) for builtin_interfaces/Time."""
  sec, nsec = divmod(int(ts), 1_000_000_000)
  return sec, nsec
