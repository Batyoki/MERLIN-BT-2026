"""Control Simulator window: parameter table from @param and the engine-ready signal (offscreen Qt)."""

import os
import sys
import time

import pytest

pytest.importorskip("PyQt5")
pytest.importorskip("matplotlib")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from PyQt5.QtWidgets import QApplication          # noqa: E402

app = QApplication.instance() or QApplication([])

from control_simulator import control_simulator_gui as gui   # noqa: E402


@pytest.fixture
def window(monkeypatch):
    # no MATLAB / Octave needed: the engine "starts" immediately
    monkeypatch.setattr(gui.MATLABControlSimulator, "start_engine", lambda self, allow_octave=True: True)
    w = gui.ControlSimulatorGUI()
    t0 = time.time()
    while w.tuner is None and time.time() - t0 < 5:
        app.processEvents()
        time.sleep(0.01)
    yield w
    w.simulator = None
    w.close()


def test_engine_ready_reaches_the_gui_thread(window):
    """The engine starts in a Python thread; its result must arrive in the GUI (signal, not QTimer)."""
    assert window.tuner is not None
    assert "engine ready" in window.statusBar().currentMessage()


def test_parameter_table_follows_the_declarations(window):
    window.control_law_edit.setPlainText(
        "% @param min=0 max=50\nKp = 8;\nKi = 0.5; % @param min=0 max=10\nu = Kp*e + Ki*integral;")
    assert list(window.param_inputs) == ["Kp", "Ki"]
    assert window.param_inputs["Kp"].value() == 8.0
    lo, hi = window.param_bounds_inputs["Ki"]
    assert (lo.value(), hi.value()) == (0.0, 10.0)
    # changing the script rebuilds the table (old widgets are removed, not left behind)
    window.control_law_edit.setPlainText("% @param min=1 max=5\nK = 2;\nu = K*e;")
    assert list(window.param_inputs) == ["K"]
    assert window.param_layout.count() == 6          # label, value, "Min:", min, "Max:", max
    window.control_law_edit.setPlainText("% @param min=5 max=1\nK = 2;\nu = K*e;")
    assert "@param error" in window.param_status_label.text()
