import os

import numpy as np
import pytest

from comma_audio.spl import SplMeter, apply_a_weighting, calculate_spl

MICD = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..", "openpilot", "system", "micd.py")


def test_silence_and_full_scale():
  assert calculate_spl(np.zeros(100)) == (0.0, 0.0)
  p, db = calculate_spl(np.ones(100))
  assert p == 1.0 and db == pytest.approx(20 * np.log10(1 / 2e-5))


def test_a_weighting_attenuates_low_frequencies():
  t = np.arange(1600) / 16000
  low, mid = np.sin(2 * np.pi * 50 * t), np.sin(2 * np.pi * 1000 * t)
  assert calculate_spl(apply_a_weighting(low, 16000))[1] < calculate_spl(apply_a_weighting(mid, 16000))[1] - 20


def test_meter_windows():
  m = SplMeter(window=1600)
  assert m.feed(np.zeros(800)) == []
  assert len(m.feed(np.zeros(2500))) == 2
  assert m.buf.size == 100


@pytest.mark.skipif(not os.path.exists(MICD), reason="openpilot micd.py not available")
def test_matches_openpilot_micd():
  # load micd's pure functions without its openpilot imports
  src = open(MICD).read().split("class Mic:")[0]
  src = "\n".join(l for l in src.splitlines() if not l.startswith("from openpilot"))
  ns: dict = {}
  exec(compile(src, "micd", "exec"), ns)
  rng = np.random.default_rng(0)
  x = rng.normal(0, 0.1, 1600)
  ref_p, _ = ns["calculate_spl"](x)
  ref_pw, ref_db = ns["calculate_spl"](ns["apply_a_weighting"](x))
  (p, pw, db), = SplMeter().feed(x)
  assert (p, pw, db) == pytest.approx((ref_p, ref_pw, ref_db))
