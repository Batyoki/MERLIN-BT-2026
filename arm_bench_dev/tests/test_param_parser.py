"""Tests for the '@param' reader and the MATLAB-free dry run of the Control Simulator."""

import os
import sys

import pytest

SIM = os.path.join(os.path.dirname(__file__), "..", "src", "control_simulator")
sys.path.insert(0, SIM)

from param_parser import ParamError, parse_control_script   # noqa: E402
from python_sim import UnsupportedLaw, simulate              # noqa: E402


def example(name):
    return open(os.path.join(SIM, "examples", name)).read()


def test_preceding_line_inline_and_named_forms():
    ps = parse_control_script(example("pd_with_feedforward.m"))
    assert [p.name for p in ps.params] == ["Kp", "Kd", "Kff"]
    assert ps.defaults() == {"Kp": 6.0, "Kd": 1.2, "Kff": 2.0}
    assert ps.bounds()["Kd"] == (0.0, 20.0)
    assert ps.params[1].desc.startswith("derivative gain")
    assert not ps.warnings


def test_units_descriptions_and_body():
    ps = parse_control_script(example("pd_position.m"))
    assert ps.params[0].unit == "N/m" and ps.params[0].desc == "proportional gain"
    assert "Kp = 8" not in ps.body and "@param" not in ps.body       # declarations are removed
    assert "u = Kp*e + Ki*integral + Kd*derivative;" in ps.body


def test_hash_comments_become_matlab_comments_in_body():
    ps = parse_control_script("# note\n# @param min=0 max=5\nK = 1;\nu = K*e;  # trailing")
    assert "%" in ps.body and "#" not in ps.body


@pytest.mark.parametrize("text, msg", [
    ("% @param min=5 max=1\nK=2;\nu=K*e;", "min < max"),
    ("% @param min=0 max=1\nK=2;\nu=K*e;", "outside"),
    ("% @param min=0 max=1\nK = e + 1;", "not a number"),
    ("% @param min=0 max=1\nK=abc;\nu=K*e;", "not followed by an assignment|not a number"),
    ("% @param min=0 max=1\ne=0.5;\nu=e;", "reserved"),
    ("K=1; % @param min=0 max=2\nK=1; % @param min=0 max=2\nu=K*e;", "twice"),
    ("% @param K min=0 max=2\nu=K*e;", "followed by an assignment to"),
    ("% @param K min=0 max=2", "neither an assignment"),
])
def test_malformed_declarations_are_rejected(text, msg):
    with pytest.raises(ParamError, match=msg):
        parse_control_script(text)


def test_missing_bounds_warn_and_unused_warns():
    ps = parse_control_script("K = 2; % @param\nJ = 1; % @param min=0 max=3\nu = K*e;")
    assert any("no min" in w for w in ps.warnings) and any("no max" in w for w in ps.warnings)
    assert ps.bounds()["K"] == (0.0, 20.0)
    assert any("'J' is declared but never used" in w for w in ps.warnings)


def test_no_annotation_means_fallback():
    ps = parse_control_script("u = Kp*e + Kd*derivative;")
    assert not ps.annotated and ps.params == []


def test_assignment_values_are_not_reapplied_over_tuned_values():
    """The annotated assignment must not run, otherwise it would reset every tuned value."""
    text = example("pd_position.m")
    a = simulate(text, {"Kp": 40.0, "Ki": 5.0, "Kd": 8.0})["metrics"]["ise"]
    b = simulate(text)["metrics"]["ise"]
    assert a < 0.5 * b


def test_worked_example_numbers():
    """docs/CONTROL_SIMULATOR_WALKTHROUGH.md quotes these (Python mirror of run_control_sim.m)."""
    text = example("pd_position.m")
    before = simulate(text)["metrics"]
    after = simulate(text, {"Kp": 45.378, "Ki": 5.981, "Kd": 9.41})["metrics"]
    assert before["overshoot_percent"] == pytest.approx(36.28, abs=0.05)
    assert after["overshoot_percent"] < 3.0 and after["itae"] < 0.1 * before["itae"]
    assert after["steady_state_error"] < 1e-2


def test_unsupported_control_flow_is_reported():
    with pytest.raises(UnsupportedLaw):
        simulate("if e > 0\n u = e;\nelse\n u = 0;\nend")


def test_reassignment_of_a_declared_parameter_is_warned():
    ps = parse_control_script("% @param min=0 max=5\nK = 1;\nK = 3;\nu = K*e;")
    assert any("assigned again" in w for w in ps.warnings)
