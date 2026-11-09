"""Static checks of the repository sources: shellcheck, ruff, clang-format and clang-tidy.

Each check is skipped when its tool is not installed. The tools are looked up in the Python
environment running the tests (the conda environment of environment.yml pins them), then on PATH.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import pytest
from conftest import DEFAULT_TIMEOUT_S, REPO_ROOT, SCRIPTS_DIR, ShellRunner

ENVIRONMENT_BIN_DIR: Path = Path(sys.prefix) / "bin"

# clang-tidy works on the compile commands of the last build of the package
COMPILE_DATABASE_DIR: Path = REPO_ROOT / "build" / "vicon_receiver"
CLANG_TIDY_TIMEOUT_S: float = 600.0


def Find_tool(name: str) -> str | None:
    """Return the path of a developer tool, or None if it is not installed."""
    candidate_ = ENVIRONMENT_BIN_DIR / name
    if candidate_.is_file():
        return str(candidate_)
    return shutil.which(name)


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
        timeout=DEFAULT_TIMEOUT_S,
        check=False,
    )
    assert result_.returncode == 0, result_.stdout + result_.stderr


@pytest.mark.parametrize("arguments", [["check"], ["format", "--check"]])
def test_python_sources_pass_ruff(arguments: list[str]) -> None:
    """All Python sources follow pyproject.toml: lint rules (typing.Any banned) and formatting."""
    ruff_ = Find_tool("ruff")
    if ruff_ is None:
        pytest.skip("ruff not installed (it is in the conda environment of environment.yml)")

    result_ = subprocess.run(
        [ruff_, *arguments, "."],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        timeout=DEFAULT_TIMEOUT_S,
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


def test_cpp_sources_pass_clang_tidy() -> None:
    """The C++ sources of the node and the probe pass .clang-tidy (vendored headers excluded)."""
    clang_tidy_ = Find_tool("clang-tidy")
    if clang_tidy_ is None:
        pytest.skip("clang-tidy not installed (it is in the conda environment of environment.yml)")
    if not (COMPILE_DATABASE_DIR / "compile_commands.json").is_file():
        pytest.skip(f"no compile commands in {COMPILE_DATABASE_DIR}: tests/run_tests.sh")

    # A library preloaded by the user's shell has no place in the analysis
    environment_ = {key_: value_ for key_, value_ in os.environ.items() if key_ != "LD_PRELOAD"}
    # The conda package has no Clang builtin headers (stddef.h): use those of the build compiler
    compiler_headers_ = subprocess.run(
        ["gcc", "-print-file-name=include"],
        env=environment_,
        capture_output=True,
        text=True,
        timeout=DEFAULT_TIMEOUT_S,
        check=True,
    ).stdout.strip()

    def _Findings(source: Path) -> str:
        """Report of clang-tidy on one source, empty if it passes."""
        result_ = subprocess.run(
            [
                clang_tidy_,
                "-p",
                str(COMPILE_DATABASE_DIR),
                "--quiet",
                f"--extra-arg=-isystem{compiler_headers_}",
                str(source),
            ],
            cwd=REPO_ROOT,
            env=environment_,
            capture_output=True,
            text=True,
            timeout=CLANG_TIDY_TIMEOUT_S,
            check=False,
        )
        return "" if result_.returncode == 0 else result_.stdout + result_.stderr

    sources_ = sorted((REPO_ROOT / "vicon_receiver" / "src").rglob("*.cpp"))
    assert sources_, "no C++ source found"
    with ThreadPoolExecutor(max_workers=os.cpu_count()) as pool_:
        findings_ = [report_ for report_ in pool_.map(_Findings, sources_) if report_]
    assert not findings_, "\n".join(findings_)
