"""The statistics of the hardware checks, and the WebXR loopback, tested on synthetic data (no hardware)."""

import numpy as np
import pytest

from arm_bench import hardware_check as hc


def test_analyse_stream_accepts_a_steady_camera_and_rejects_a_stalled_one():
    steady = np.arange(0, 3, 1 / 15)
    ok = hc.analyse_stream(steady, (480, 640, 3), 15)
    assert ok["ok"] and ok["fps"] == pytest.approx(15, rel=0.02) and ok["jitter_ms"] < 1e-6
    stalled = np.concatenate([np.arange(0, 1, 1 / 15), np.arange(2, 3, 1 / 15)])     # 1 s hole
    bad = hc.analyse_stream(stalled, (480, 640, 3), 15)
    assert not bad["ok"] and bad["max_gap_ms"] > 900
    assert not hc.analyse_stream([0.0, 0.1], None)["ok"]


def test_camera_skew_is_the_offset_between_streams():
    a = np.arange(0, 2, 1 / 15)
    assert hc.camera_skew_ms(a, a + 0.020) == pytest.approx(20, abs=0.01)
    assert hc.camera_skew_ms(a, a) == 0.0


def test_joystick_idle_offset_and_range():
    t = np.linspace(0, 1, 500)
    drift = np.where(t[:, None] < 0.3, 0.02, np.sin(8 * t)[:, None])          # quiet, then swept
    r = hc.analyse_axes(np.hstack([drift, drift * 0 + 0.3]))                     # 2nd axis: stuck at 0.3
    assert r["idle_ok"] is False                                                # stuck axis exceeds the 0.1 dead-zone
    assert list(r["moved"]) == [True, False]
    assert hc.analyse_axes(drift)["idle_ok"]


def test_webxr_loopback_runs_the_real_handler():
    pytest.importorskip("fastapi")
    pytest.importorskip("httpx")
    assert hc.check_webxr_loopback(n_msgs=10)
