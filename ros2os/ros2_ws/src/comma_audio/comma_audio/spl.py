"""Sound pressure level computations from openpilot's micd (no ROS)."""
from functools import cache

import numpy as np

REFERENCE_SPL = 2e-5  # N/m^2


@cache
def a_weighting_filter(n: int, sample_rate: int) -> np.ndarray:
  # https://en.wikipedia.org/wiki/A-weighting
  f = np.fft.fftfreq(n, d=1 / sample_rate)
  a = 12194 ** 2 * f ** 4 / ((f ** 2 + 20.6 ** 2) * (f ** 2 + 12194 ** 2) * np.sqrt((f ** 2 + 107.7 ** 2) * (f ** 2 + 737.9 ** 2)))
  return a / np.max(a)


def calculate_spl(x: np.ndarray) -> tuple[float, float]:
  """(RMS amplitude, level in dB re 20 uPa). Uncalibrated: the microphone gain is unknown."""
  p = float(np.sqrt(np.mean(x ** 2)))
  return p, (20 * np.log10(p / REFERENCE_SPL) if p > 0 else 0.0)


def apply_a_weighting(x: np.ndarray, sample_rate: int) -> np.ndarray:
  windowed = x * np.hanning(len(x))
  return np.abs(np.fft.ifft(np.fft.fft(windowed) * a_weighting_filter(len(x), sample_rate)))


class SplMeter:
  """Accumulates float samples and yields an A-weighted level for every full window."""

  def __init__(self, window: int = 1600, sample_rate: int = 16000):
    self.window = window
    self.sample_rate = sample_rate
    self.buf = np.empty(0)

  def feed(self, samples: np.ndarray) -> list[tuple[float, float, float]]:
    """Returns [(pressure, weighted pressure, weighted level dB)] for each completed window."""
    self.buf = np.concatenate((self.buf, samples))
    out = []
    while self.buf.size >= self.window:
      w, self.buf = self.buf[:self.window], self.buf[self.window:]
      p, _ = calculate_spl(w)
      pw, dbw = calculate_spl(apply_a_weighting(w, self.sample_rate))
      out.append((p, pw, dbw))
    return out
