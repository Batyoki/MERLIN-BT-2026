"""
MATLAB-free dry run of the Control Simulator.

Mirrors ``matlab_scripts/run_control_sim.m`` (same plant, 10 ms step, explicit Euler,
|u| <= 10, same reference generator) and evaluates a control law written in the
scalar subset of MATLAB used by the examples:

    assignments, + - * / ^, parentheses, sin cos tan exp log sqrt abs max min sign,
    pi, comments with %, trailing semicolons, `if/else/end` is NOT supported.

Purpose: check a script (its @param header, its gains) and its worked-example numbers
without a MATLAB licence, e.g. in CI. It is not a replacement for the MATLAB window:
a law outside the subset raises ``UnsupportedLaw`` instead of guessing.
"""

from __future__ import annotations

import math
import re
from typing import Dict, Optional

import numpy as np

try:
    from .param_parser import parse_control_script
    from .matlab_engine_interface import MATLABControlSimulator
except ImportError:                                           # run as plain module
    from param_parser import parse_control_script
    from matlab_engine_interface import MATLABControlSimulator

DT = 0.01
U_MAX = 10.0
M, B_DAMP, K_SPRING = 1.0, 0.5, 2.0                           # run_control_sim.m


class UnsupportedLaw(ValueError):
    pass


_ALLOWED = {"sin": math.sin, "cos": math.cos, "tan": math.tan, "exp": math.exp, "log": math.log,
            "sqrt": math.sqrt, "abs": abs, "max": max, "min": min,
            "sign": lambda v: (v > 0) - (v < 0), "pi": math.pi}


def _to_python(law: str) -> str:
    out = []
    for raw in law.splitlines():
        s = re.sub(r"%.*", "", raw).strip()
        if not s:
            continue
        if re.match(r"^(if|else|elseif|for|while|end|switch|case|function)\b", s):
            raise UnsupportedLaw(f"control flow '{s.split()[0]}' is outside the supported subset")
        s = s.rstrip(";").replace(".*", "*").replace("./", "/").replace(".^", "**").replace("^", "**")
        out.append(s)
    return "\n".join(out)


def _reference(time: np.ndarray, kind: str, amp: float) -> np.ndarray:
    kind = kind.lower()
    if kind == "ramp":
        return amp * time / np.max(time)
    if kind == "sine":
        return amp * np.sin(2 * np.pi * 0.5 * time)
    if kind == "square":
        return amp * (2.0 * (np.sin(2 * np.pi * 0.2 * time) >= 0) - 1.0)
    return amp * np.ones_like(time)


def simulate(law_text: str, parameters: Optional[Dict[str, float]] = None, sim_time: float = 10.0,
             reference_signal: str = "step", reference_amplitude: float = 1.0) -> Dict:
    """Run the plant of run_control_sim.m. ``law_text`` may contain @param declarations; their
    defaults are used for every parameter that is not given in ``parameters``."""
    ps = parse_control_script(law_text)
    body = ps.body if ps.annotated else law_text
    values = dict(ps.defaults())
    values.update(parameters or {})
    code = compile(_to_python(body), "<law>", "exec")

    time = np.arange(0.0, sim_time + 1e-12, DT)
    ref = _reference(time, reference_signal, reference_amplitude)
    A = np.array([[0.0, 1.0], [-K_SPRING / M, -B_DAMP / M]])
    Bv = np.array([0.0, 1.0 / M])
    x = np.zeros(2)
    n = len(time)
    out, ctl, err, st = np.zeros(n), np.zeros(n), np.zeros(n), np.zeros((2, n))
    integral, prev_e = 0.0, 0.0
    for i in range(n):
        y = x[0]
        e = ref[i] - y
        integral += e * DT
        derivative = (e - prev_e) / DT
        env = {"__builtins__": {}, **_ALLOWED, **values, "e": e, "dt": DT, "integral": integral,
               "derivative": derivative, "x": x.copy(), "r": ref[i], "t": time[i], "u": 0.0}
        try:
            exec(code, env)                                              # noqa: S102 (restricted namespace)
        except Exception as ex:
            raise UnsupportedLaw(f"cannot evaluate the law in Python: {ex}") from ex
        u = float(np.clip(env["u"], -U_MAX, U_MAX))
        st[:, i], out[i], ctl[i], err[i] = x, y, u, e
        prev_e = e
        x = x + (A @ x + Bv * u) * DT
    res = dict(time=time, states=out, control=ctl, reference=ref, error=err, full_states=st)
    res["metrics"] = MATLABControlSimulator._compute_metrics(None, res)
    return res


class PythonSimulator:
    """Drop-in for MATLABControlSimulator for ParameterTuner / tests (no MATLAB needed)."""

    def is_engine_running(self) -> bool:
        return True

    def run_control_simulation(self, algorithm_name="pid", parameters=None, sim_time=10.0, initial_state=None,
                               reference_signal="step", reference_amplitude=1.0, control_law_text=None):
        return simulate(control_law_text or "u = 0;", parameters, sim_time, reference_signal, reference_amplitude)
