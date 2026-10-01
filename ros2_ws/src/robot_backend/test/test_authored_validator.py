"""validate_source must accept the allowed subset and reject everything else.

One test per rule, in both directions where a rule has an accept side: the
validator is the first of the three sandbox layers (validator, builtins
whitelist, budget), and a rule it silently stopped enforcing would only show
up as authored code doing something it must not.
"""

from __future__ import annotations

import pytest

from robot_backend.watchdog.authored import (
    MAX_SOURCE_CHARS,
    AuthoredCodeError,
    validate_source,
)


def _rejects(source: str) -> AuthoredCodeError:
    with pytest.raises(AuthoredCodeError) as exc_info:
        validate_source(source)
    assert exc_info.value.stage == "validate"
    return exc_info.value


# --- accepted ---------------------------------------------------------------

def test_minimal_check_is_accepted():
    validate_source("def check(ctx):\n    return None\n")


def test_helper_functions_are_accepted():
    validate_source(
        "def helper(x):\n"
        "    return x * 2\n"
        "def check(ctx):\n"
        "    return helper(1) and None\n"
    )


def test_top_level_literal_assignments_are_accepted():
    validate_source(
        "EPS = 0.02\n"
        "NEG = -3\n"
        "NAME = 'finger_joint'\n"
        "AXES = ('x', 'y', 'z')\n"
        "BOUNDS = {'lo': 0.0, 'hi': 1.0}\n"
        "def check(ctx):\n"
        "    return None\n"
    )


def test_for_loop_and_bare_except_are_accepted():
    validate_source(
        "def check(ctx):\n"
        "    total = 0\n"
        "    for row in ctx.joint_window('finger_joint', 1.0):\n"
        "        try:\n"
        "            total = total + abs(row['effort'])\n"
        "        except:\n"
        "            pass\n"
        "    return None\n"
    )


def test_math_use_is_accepted():
    validate_source(
        "def check(ctx):\n"
        "    return None if math.isfinite(1.0) else {'value': 1.0}\n"
    )


# --- size and shape ---------------------------------------------------------

def test_empty_source_is_rejected():
    _rejects("")


def test_oversized_source_is_rejected():
    # Padded with comments so the size cap is the only rule the source hits.
    source = "def check(ctx):\n    return None\n" + "# pad\n" * (
        MAX_SOURCE_CHARS // 6 + 1
    )
    assert len(source) > MAX_SOURCE_CHARS
    _rejects(source)


def test_syntax_error_is_rejected_with_lineno():
    err = _rejects("def check(ctx)\n    return None\n")
    assert err.lineno == 1


def test_top_level_expression_is_rejected():
    _rejects("1 + 1\ndef check(ctx):\n    return None\n")


def test_top_level_non_literal_assignment_is_rejected():
    _rejects("EPS = abs(-1)\ndef check(ctx):\n    return None\n")


def test_missing_check_is_rejected():
    _rejects("def helper(x):\n    return x\n")


def test_two_checks_are_rejected():
    _rejects(
        "def check(ctx):\n    return None\n"
        "def check(ctx):\n    return None\n"
    )


def test_check_with_zero_args_is_rejected():
    _rejects("def check():\n    return None\n")


def test_check_with_two_args_is_rejected():
    _rejects("def check(ctx, extra):\n    return None\n")


def test_check_with_varargs_is_rejected():
    _rejects("def check(*args):\n    return None\n")


def test_check_with_default_is_rejected():
    _rejects("def check(ctx=None):\n    return None\n")


# --- forbidden statements ---------------------------------------------------

def test_import_is_rejected():
    _rejects("import os\ndef check(ctx):\n    return None\n")


def test_import_from_is_rejected():
    _rejects("from os import path\ndef check(ctx):\n    return None\n")


def test_import_inside_check_is_rejected():
    _rejects("def check(ctx):\n    import os\n    return None\n")


def test_while_is_rejected():
    _rejects("def check(ctx):\n    while True:\n        pass\n")


def test_global_is_rejected():
    _rejects("X = 1\ndef check(ctx):\n    global X\n    return None\n")


def test_nonlocal_is_rejected():
    _rejects(
        "def check(ctx):\n"
        "    x = 1\n"
        "    def inner():\n"
        "        nonlocal x\n"
        "        x = 2\n"
        "    return None\n"
    )


def test_class_is_rejected():
    _rejects("class Foo:\n    pass\ndef check(ctx):\n    return None\n")


def test_async_def_is_rejected():
    _rejects("async def helper():\n    return 1\ndef check(ctx):\n    return None\n")


def test_yield_is_rejected():
    _rejects("def helper():\n    yield 1\ndef check(ctx):\n    return None\n")


def test_yield_from_is_rejected():
    _rejects(
        "def helper():\n    yield from (1, 2)\ndef check(ctx):\n    return None\n"
    )


# --- forbidden names --------------------------------------------------------

def test_underscore_name_is_rejected():
    _rejects("def check(ctx):\n    _x = 1\n    return None\n")


def test_bare_underscore_loop_variable_is_allowed():
    validate_source(
        "def check(ctx):\n"
        "    total = 0\n"
        "    for _ in range(3):\n"
        "        total = total + 1\n"
        "    return None\n"
    )


def test_dunder_import_call_is_rejected():
    err = _rejects("def check(ctx):\n    return __import__('os')\n")
    assert "__import__" in str(err)


def test_getattr_dunder_class_is_rejected():
    _rejects("def check(ctx):\n    return getattr(ctx, '__class__')\n")


def test_tuple_dunder_class_is_rejected():
    _rejects("def check(ctx):\n    return ().__class__\n")


def test_dunder_subclasses_and_globals_are_rejected():
    _rejects("def check(ctx):\n    return ctx.__subclasses__\n")
    _rejects("def check(ctx):\n    return check.__globals__\n")
    _rejects("def check(ctx):\n    return __builtins__\n")


def test_dunder_name_is_rejected():
    _rejects("def check(ctx):\n    return __builtins__\n")


def test_underscore_attribute_is_rejected():
    _rejects("def check(ctx):\n    return ctx._recorder\n")


def test_dunder_attribute_is_rejected():
    _rejects("def check(ctx):\n    return ctx.__class__\n")


@pytest.mark.parametrize("name", [
    "eval", "exec", "compile", "open", "getattr", "setattr", "delattr",
    "globals", "locals", "vars", "dir", "input", "breakpoint", "help",
    "memoryview", "type", "super", "object",
])
def test_forbidden_builtin_names_are_rejected(name):
    err = _rejects(f"def check(ctx):\n    return {name}\n")
    assert name in str(err)


def test_forbidden_name_as_call_is_rejected():
    _rejects("def check(ctx):\n    return eval('1')\n")


def test_rejection_carries_the_line_number():
    err = _rejects("def check(ctx):\n    x = 1\n    return eval('1')\n")
    assert err.lineno == 3
