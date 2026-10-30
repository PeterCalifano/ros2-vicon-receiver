"""Static checks of the repository sources: shell scripts (shellcheck) and C++ formatting.

Each check is skipped when its tool is not installed. The tools are looked up on PATH and in the
optional developer virtual environment build/.devtools (pip: clang-format, shellcheck-py).
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from conftest import REPO_ROOT, SCRIPTS_DIR, ShellRunner

DEVTOOLS_BIN_DIR: Path = REPO_ROOT / "build" / ".devtools" / "bin"


def Find_tool(name: str) -> str | None:
    """Return the path of a developer tool, or None if it is not installed."""
    found_ = shutil.which(name)
    if found_ is not None:
        return found_
    candidate_ = DEVTOOLS_BIN_DIR / name
    if candidate_.is_file():
        return str(candidate_)
    return None


def Shell_sources() -> list[Path]:
    """All bash sources of the repository (scripts and test fixtures)."""
    sources_ = sorted(SCRIPTS_DIR.glob("*.sh"))
    sources_ += sorted((SCRIPTS_DIR / "lib").glob("*.sh"))
    sources_ += sorted((REPO_ROOT / "tests" / "fixtures" / "fake_bin").iterdir())
    sources_.append(REPO_ROOT / "tests" / "run_tests.sh")
    return [source_ for source_ in sources_ if source_.is_file()]


def test_shell_sources_pass_shellcheck() -> None:
    """All bash sources are free of shellcheck warnings."""
    shellcheck_ = Find_tool("shellcheck")
    if shellcheck_ is None:
        pytest.skip("shellcheck not installed (sudo apt install shellcheck)")

    sources_ = [str(source_) for source_ in Shell_sources()]
    result_ = subprocess.run(
        [shellcheck_, "-x", *sources_],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result_.returncode == 0, result_.stdout + result_.stderr


def test_cpp_sources_are_formatted(shell: ShellRunner) -> None:
    """All C++ sources follow .clang-format (scripts/format.sh --check)."""
    clang_format_ = Find_tool("clang-format")
    if clang_format_ is None:
        pytest.skip("clang-format not installed (sudo apt install clang-format)")

    result_ = shell.run_script("format.sh", ["--check"], extra_env={"CLANG_FORMAT": clang_format_})
    assert result_.returncode == 0, result_.stdout + result_.stderr
