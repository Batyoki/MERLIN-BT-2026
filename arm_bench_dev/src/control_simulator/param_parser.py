"""
Read tunable parameters from a control script.

A control script is the text typed into (or loaded into) the "Control law" box of
the Control Simulator. Parameters are declared with an annotation comment, so that
the GUI, the tuner and the MATLAB engine all read the same single source of truth:

    % @param min=0 max=50 desc="proportional gain"
    Kp = 8.0;

    Ki = 0.5;   % @param min=0 max=10 desc="integral gain" unit=1/s

    % @param Kd default=0.4 min=0 max=5          (declaration without assignment)

    u = Kp*e + Ki*integral + Kd*derivative;

Rules
-----
* The comment marker may be ``%`` (MATLAB), ``#`` or ``//``; the tag is ``@param``.
* Options are ``key=value`` pairs: ``min``, ``max``, ``default``, ``step``, ``unit``
  and ``desc`` (quote values that contain spaces). Unknown keys are kept in
  ``ParamSpec.extra`` and reported as a warning.
* An annotation applies to the assignment ``name = number;`` on the SAME line, or,
  if the annotation stands alone on a line, to the next non-blank, non-comment
  line. ``@param NAME ...`` names the parameter explicitly and needs no assignment.
* The default is the assigned number (or ``default=``). If ``min``/``max`` are
  missing they are derived from the default (0 .. 10x, or +-10x for a negative one)
  and a warning is emitted, so that nothing is silently unbounded.
* The annotated assignment lines are removed from the executable body: the values
  are injected by the simulator (so that the GUI spin boxes and the tuner override
  them). Without this, the assignment in the script would overwrite every tuned value.
* A script without any ``@param`` falls back to the old heuristic (every free
  identifier is a parameter, see ``matlab_engine_interface.extract_parameter_names``).

Nothing here depends on MATLAB or Qt.
"""

from __future__ import annotations

import ast
import math
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

#: names provided by the simulator inside every control law
RESERVED = {"u", "e", "dt", "integral", "derivative", "x", "r", "t", "i", "k",
            "sin", "cos", "tan", "exp", "log", "sqrt", "abs", "max", "min", "sign",
            "pi", "inf", "nan"}

_TAG = re.compile(r"(?P<marker>%|#|//)\s*@param\b(?P<rest>.*)$")
_ASSIGN = re.compile(r"^\s*(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*=\s*(?P<value>[^;%#]*?)\s*;?\s*(?:(?:%|#|//).*)?$")
_NUM = re.compile(r"^[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?$")
_IDENT = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_OPT = re.compile(r'(\w+)\s*=\s*("[^"]*"|\'[^\']*\'|[^\s]+)')
_KNOWN = {"min", "max", "default", "step", "unit", "desc"}


@dataclass
class ParamSpec:
    name: str
    default: float
    min: float
    max: float
    step: Optional[float] = None
    unit: str = ""
    desc: str = ""
    line: int = 0
    extra: Dict[str, str] = field(default_factory=dict)

    @property
    def bounds(self) -> Tuple[float, float]:
        return self.min, self.max


@dataclass
class ParsedScript:
    params: List[ParamSpec]
    body: str                       # executable text without the declarations
    warnings: List[str]
    annotated: bool                 # False: no @param found, caller should use its fallback

    def defaults(self) -> Dict[str, float]:
        return {p.name: p.default for p in self.params}

    def bounds(self) -> Dict[str, Tuple[float, float]]:
        return {p.name: (p.min, p.max) for p in self.params}


class ParamError(ValueError):
    pass


def _number(text: str, what: str, line: int) -> float:
    text = text.strip()
    if not _NUM.match(text):
        # allow simple constant expressions such as 2*pi or 1e-3/2 (no names other than pi)
        try:
            node = ast.parse(text.replace("^", "**").replace("pi", f"{math.pi!r}"), mode="eval")
            if not all(isinstance(n, (ast.Expression, ast.BinOp, ast.UnaryOp, ast.Constant, ast.operator,
                                      ast.unaryop)) for n in ast.walk(node)):
                raise ValueError
            return float(eval(compile(node, "<param>", "eval"), {"__builtins__": {}}))
        except Exception:
            raise ParamError(f"line {line}: {what} '{text}' is not a number")
    return float(text)


def _options(rest: str, line: int, warnings: List[str]) -> Tuple[Optional[str], Dict[str, str]]:
    """Split the text after '@param' into an optional leading name and key=value options."""
    rest = rest.strip()
    name = None
    m = re.match(r"([A-Za-z_][A-Za-z0-9_]*)(?=\s|$)", rest)
    if m and "=" not in m.group(1):
        name = m.group(1)
        rest = rest[m.end():]
    opts: Dict[str, str] = {}
    for key, val in _OPT.findall(rest):
        opts[key.lower()] = val.strip("\"'")
    leftover = _OPT.sub("", rest).strip()
    if leftover:
        warnings.append(f"line {line}: ignored text '{leftover}' in @param annotation")
    return name, opts


def _code_part(line: str) -> str:
    """Line without its trailing comment."""
    return re.split(r"%|#|//", line, maxsplit=1)[0]


def parse_control_script(text: str) -> ParsedScript:
    """Parse ``text``. Raises ParamError for malformed declarations (bad number, duplicate name,
    min >= max, default outside [min, max], reserved or invalid names)."""
    lines = (text or "").splitlines()
    warnings: List[str] = []
    specs: Dict[str, ParamSpec] = {}
    drop = set()                    # line indices removed from the body
    pending: Optional[Tuple[int, Optional[str], Dict[str, str]]] = None
    annotated = False

    def finish(name: str, opts: Dict[str, str], assigned: Optional[str], ln: int):
        if not _IDENT.match(name):
            raise ParamError(f"line {ln}: '{name}' is not a valid parameter name")
        if name in RESERVED:
            raise ParamError(f"line {ln}: '{name}' is reserved by the simulator and cannot be a parameter")
        if name in specs:
            raise ParamError(f"line {ln}: parameter '{name}' is declared twice (first on line {specs[name].line})")
        if assigned is not None and "default" in opts and abs(_number(assigned, "value", ln) - _number(opts["default"], "default", ln)) > 0:
            warnings.append(f"line {ln}: '{name}' has both an assignment and default=; the assignment wins")
        if assigned is not None:
            default = _number(assigned, "value", ln)
        elif "default" in opts:
            default = _number(opts["default"], "default", ln)
        else:
            raise ParamError(f"line {ln}: parameter '{name}' has neither an assignment nor default=")
        if "min" in opts:
            lo = _number(opts["min"], "min", ln)
        else:
            lo = 0.0 if default >= 0 else -10.0 * abs(default)
            warnings.append(f"line {ln}: '{name}' has no min, using {lo:g}")
        if "max" in opts:
            hi = _number(opts["max"], "max", ln)
        else:
            hi = 10.0 * abs(default) if default != 0 else 1.0
            warnings.append(f"line {ln}: '{name}' has no max, using {hi:g}")
        if not lo < hi:
            raise ParamError(f"line {ln}: '{name}' needs min < max (got {lo:g}, {hi:g})")
        if not lo <= default <= hi:
            raise ParamError(f"line {ln}: default {default:g} of '{name}' lies outside [{lo:g}, {hi:g}]")
        step = _number(opts["step"], "step", ln) if "step" in opts else None
        extra = {k: v for k, v in opts.items() if k not in _KNOWN}
        if extra:
            warnings.append(f"line {ln}: unknown @param option(s) {sorted(extra)} for '{name}' (kept in .extra)")
        specs[name] = ParamSpec(name, default, lo, hi, step, opts.get("unit", ""), opts.get("desc", ""), ln, extra)

    for idx, raw in enumerate(lines):
        ln = idx + 1
        tag = _TAG.search(raw)
        code = _code_part(raw).strip()
        if tag:
            annotated = True
            name, opts = _options(tag.group("rest"), ln, warnings)
            if code:                                     # inline form:  Kp = 2;  % @param ...
                m = _ASSIGN.match(raw)
                if not m:
                    raise ParamError(f"line {ln}: @param on a line that is not 'name = number;'")
                finish(name or m.group("name"), opts, m.group("value"), ln)
                drop.add(idx)
            elif name:                                   # standalone with explicit name
                if "default" not in opts:
                    # may still be followed by an assignment line; handled below via pending
                    pending = (idx, name, opts)
                    drop.add(idx)
                    continue
                finish(name, opts, None, ln)
                drop.add(idx)
            else:                                        # applies to the next code line
                pending = (idx, None, opts)
                drop.add(idx)
            continue
        if pending is not None:
            if not code:                                 # blank or ordinary comment: keep waiting
                continue
            p_idx, p_name, p_opts = pending
            m = _ASSIGN.match(raw)
            if not m:
                raise ParamError(f"line {p_idx + 1}: @param is not followed by an assignment 'name = number;'")
            if p_name and p_name != m.group("name"):
                raise ParamError(f"line {p_idx + 1}: @param {p_name} is followed by an assignment to '{m.group('name')}'")
            finish(m.group("name"), p_opts, m.group("value"), p_idx + 1)
            drop.add(idx)
            pending = None
    if pending is not None:
        p_idx, p_name, p_opts = pending
        if p_name and "default" in p_opts:
            pass
        elif p_name:
            raise ParamError(f"line {p_idx + 1}: parameter '{p_name}' has neither an assignment nor default=")
        else:
            raise ParamError(f"line {p_idx + 1}: @param at the end of the script has nothing to apply to")

    body_lines = []
    for idx, raw in enumerate(lines):
        if idx in drop:
            continue
        if "'" not in raw and '"' not in raw:        # MATLAB only knows '%'
            raw = raw.replace("//", "%").replace("#", "%")
        body_lines.append(raw)
    body = "\n".join(body_lines).strip("\n")
    if annotated:
        used = set(re.findall(r"\b[A-Za-z_][A-Za-z0-9_]*\b", re.sub(r"(%|#|//).*", "", body)))
        for n in specs:
            if n not in used:
                warnings.append(f"parameter '{n}' is declared but never used in the control law")
            if re.search(rf"^\s*{re.escape(n)}\s*=[^=]", re.sub(r"(%|#|//).*", "", body), re.M):
                warnings.append(f"'{n}' is assigned again in the control law; that assignment would overwrite the "
                                f"tuned value, remove it")
    return ParsedScript(list(specs.values()), body, warnings, annotated)


def split_for_simulation(law_text: str) -> Tuple[str, Dict[str, float]]:
    """(executable law, default parameter values) for a law that may or may not contain @param."""
    ps = parse_control_script(law_text)
    return (ps.body, ps.defaults()) if ps.annotated else (law_text, {})
