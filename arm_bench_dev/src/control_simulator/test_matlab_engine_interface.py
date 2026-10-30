import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

matlab_module = types.ModuleType("matlab")
engine_module = types.ModuleType("matlab.engine")


def _matlab_double(values):
    return list(values)


matlab_module.double = _matlab_double
engine_module.start_matlab = lambda: None
matlab_module.engine = engine_module
sys.modules.setdefault("matlab", matlab_module)
sys.modules.setdefault("matlab.engine", engine_module)

from arm_bench_dev.src.control_simulator.matlab_engine_interface import extract_parameter_names


def test_extract_parameter_names_from_control_law():
    law_text = "u = Kp*e + Ki*integral + Kd*derivative + Kp*e;"

    assert extract_parameter_names(law_text) == ["Kp", "Ki", "Kd"]


def test_extract_parameter_names_ignores_reserved_words():
    law_text = "u = e + dt + integral + derivative + x + r + t;"

    assert extract_parameter_names(law_text) == []
