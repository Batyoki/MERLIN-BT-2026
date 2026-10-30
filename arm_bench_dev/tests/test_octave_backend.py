"""The MATLAB scripts of the Control Simulator, executed in GNU Octave, against the Python mirror.

Skipped when octave-cli or oct2py is not installed. Uses the same entry point as the GUI
(MATLABControlSimulator with the Octave fallback), so it also covers the engine adapter.
"""

import os
import shutil
import sys

import numpy as np
import pytest

SIM = os.path.join(os.path.dirname(__file__), "..", "src", "control_simulator")
sys.path.insert(0, SIM)

pytest.importorskip("oct2py")
if shutil.which("octave-cli") is None and shutil.which("octave") is None:
    pytest.skip("GNU Octave is not installed", allow_module_level=True)

from matlab_engine_interface import MATLABControlSimulator   # noqa: E402
from python_sim import simulate                              # noqa: E402


@pytest.fixture(scope="module")
def octave_sim():
    sim = MATLABControlSimulator()
    assert sim.start_engine()
    assert sim.backend in ("MATLAB", "GNU Octave")
    yield sim
    sim.stop_engine()


def law():
    return open(os.path.join(SIM, "examples", "pd_position.m")).read()


@pytest.mark.parametrize("signal", ["step", "sine", "ramp", "square"])
def test_m_scripts_match_python_mirror(octave_sim, signal):
    params = {"Kp": 45.378, "Ki": 5.981, "Kd": 9.41}
    a = octave_sim.run_control_simulation("pid", params, 5.0, reference_signal=signal, control_law_text=law())
    b = simulate(law(), params, 5.0, signal)
    assert np.max(np.abs(np.asarray(a["states"]).ravel() - b["states"])) < 1e-9
    assert a["metrics"]["itae"] == pytest.approx(b["metrics"]["itae"], rel=1e-9)


def test_parameter_values_keep_full_precision(octave_sim):
    """eval_custom_law.m assigns the numbers themselves; num2str would keep ~5 significant digits."""
    text = "% @param min=0 max=1\nK = 0.5;\nu = 1000*K;"
    a = octave_sim.run_control_simulation("pid", {"K": 0.00123456789}, 0.05, control_law_text=text)
    assert np.asarray(a["control"]).ravel()[0] == pytest.approx(1.23456789, rel=1e-12)


def test_declared_defaults_are_used_when_no_value_is_given(octave_sim):
    a = octave_sim.run_control_simulation("pid", {}, 10.0, control_law_text=law())
    assert a["metrics"]["overshoot_percent"] == pytest.approx(36.28, abs=0.01)
