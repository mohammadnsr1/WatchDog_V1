"""Regenerate every derived prompt and task file from the S1 sources.

Run after editing watchdog_system.txt, executor_system.txt, prompt_variants.py
or a *.watchdog.yaml:

    python3 agents/prompts/regenerate_variants.py

Writes (all under agents/):
    prompts/watchdog_system_s2.txt, prompts/watchdog_system_s3.txt,
    prompts/executor_system_s4.txt   -- builder output, byte for byte;
    tasks/fmb_assembly_s4.executor.yaml, tasks/fmb_assembly_2_s4.executor.yaml
        -- their instruction_spec and object_ids block scalars are rewritten
        (to s4_task_expected's values) and their steps: block is copied
        VERBATIM from the source executor task (the derivation pins
        s4.steps == source.steps, and a source edit that is not copied over
        -- e.g. commit c390b1d commenting out a target_offset -- leaves the
        S4 file failing test_s4_task_matches_derivation); every other line,
        including comments, is left byte-identical.

test_prompt_variants.py pins each written file to exactly this derivation, so
running this script is the one sanctioned way to change a derived file.
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml

PROMPTS = Path(__file__).resolve().parent
TASKS = PROMPTS.parent / "tasks"

sys.path.insert(0, str(PROMPTS))
import prompt_variants as pv  # noqa: E402

S4_TASK_SOURCES = {
    "fmb_assembly_s4.executor.yaml": (
        "fmb_assembly.executor.yaml", "fmb_assembly.watchdog.yaml"
    ),
    "fmb_assembly_2_s4.executor.yaml": (
        "fmb_assembly_2.executor.yaml", "fmb_assembly_2.watchdog.yaml"
    ),
    # S4 versions of the Class 2/3 task variants (2026-10-21): the S1-S3
    # variant file (steps, step_plan and its own task text, unchanged) plus the
    # same two S4 blocks. The WatchDog task file is never varied. r3 is not a
    # task variant (registry edit), so it has no S4 file.
    **{
        f"runs/fmb_assembly_2_s4.{r}.executor.yaml": (
            f"runs/fmb_assembly_2.{r}.executor.yaml", "fmb_assembly_2.watchdog.yaml"
        )
        for r in ("r2", "r4", "r5", "r6")
    },
}


def _write(path: Path, text: str) -> None:
    old = path.read_text(encoding="utf-8") if path.exists() else None
    if old == text:
        print(f"unchanged  {path.name}")
        return
    path.write_text(text, encoding="utf-8")
    print(f"WROTE      {path.name}")


def _replace_block_scalar(text: str, key: str, new_value: str) -> str:
    """Replace the body of a top-level `key: |` block scalar with new_value.

    The body is every subsequent line that is blank or indented; it ends at
    the first non-indented, non-blank line. new_value is re-indented by two
    spaces, matching the files' uniform block style.
    """
    lines = text.split("\n")
    try:
        start = next(
            i for i, line in enumerate(lines) if line.startswith(f"{key}: |")
        )
    except StopIteration:
        raise ValueError(f"no block scalar {key!r} found") from None
    end = len(lines)
    for i in range(start + 1, len(lines)):
        if lines[i] and not lines[i].startswith(" "):
            end = i
            break
    # Content lines are kept VERBATIM under the two-space indent (a source
    # line may carry a trailing space the parse round-trip must preserve);
    # only blank lines stay truly blank rather than becoming "  ".
    body = [
        ("  " + line) if line else "" for line in new_value.rstrip("\n").split("\n")
    ]
    return "\n".join(lines[: start + 1] + body + [""] + lines[end:])


def _block_span(lines: list[str], key: str) -> tuple[int, int]:
    """(start, end) line indices of a top-level block: its `key:` line through
    the last line before the next non-indented, non-blank line."""
    try:
        start = next(i for i, line in enumerate(lines) if line.startswith(f"{key}:"))
    except StopIteration:
        raise ValueError(f"no top-level block {key!r} found") from None
    end = len(lines)
    for i in range(start + 1, len(lines)):
        if lines[i] and not lines[i].startswith(" ") and not lines[i].startswith("#"):
            end = i
            break
    return start, end


def _copy_block_verbatim(dest_text: str, source_text: str, key: str) -> str:
    """Replace dest's top-level `key:` block with source's, byte for byte."""
    dest_lines = dest_text.split("\n")
    source_lines = source_text.split("\n")
    d0, d1 = _block_span(dest_lines, key)
    s0, s1 = _block_span(source_lines, key)
    return "\n".join(dest_lines[:d0] + source_lines[s0:s1] + dest_lines[d1:])


def main() -> None:
    s1_watchdog = (PROMPTS / "watchdog_system.txt").read_text(encoding="utf-8")
    s1_executor = (PROMPTS / "executor_system.txt").read_text(encoding="utf-8")

    _write(PROMPTS / "watchdog_system_s2.txt", pv.build_watchdog(s1_watchdog, "S2"))
    _write(PROMPTS / "watchdog_system_s3.txt", pv.build_watchdog(s1_watchdog, "S3"))
    _write(PROMPTS / "executor_system_s4.txt", pv.build_executor_s4(s1_executor))

    for s4_name, (exec_name, wd_name) in S4_TASK_SOURCES.items():
        exec_data = yaml.safe_load((TASKS / exec_name).read_text(encoding="utf-8"))
        wd_data = yaml.safe_load((TASKS / wd_name).read_text(encoding="utf-8"))
        expected = pv.s4_task_expected(exec_data, wd_data)
        s4_path = TASKS / s4_name
        # a new S4 file starts from its source executor file, so every key and
        # comment the derivation does not touch is the source's, byte for byte
        text = (s4_path if s4_path.exists() else TASKS / exec_name).read_text(encoding="utf-8")
        text = _replace_block_scalar(
            text, "instruction_spec", expected["instruction_spec"]
        )
        text = _replace_block_scalar(text, "object_ids", expected["object_ids"])
        text = _copy_block_verbatim(
            text, (TASKS / exec_name).read_text(encoding="utf-8"), "steps"
        )
        # Round-trip check: the file must parse back to exactly the expected
        # blocks, or the write is refused.
        parsed = yaml.safe_load(text)
        for key in ("instruction_spec", "object_ids"):
            if parsed[key].rstrip() != expected[key].rstrip():
                raise RuntimeError(
                    f"{s4_name}: {key} does not round-trip; not writing."
                )
        if parsed["steps"] != exec_data["steps"]:
            raise RuntimeError(f"{s4_name}: steps does not round-trip; not writing.")
        _write(s4_path, text)


if __name__ == "__main__":
    main()
