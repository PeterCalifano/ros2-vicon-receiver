"""Unit tests of the command-line interface of the scripts in scripts/.

Only argument handling and early validation are tested here: nothing is installed, built or
launched. The end-to-end behaviour of launch.sh is covered by tests/integration.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from conftest import ShellRunner

# Scripts with a command-line interface (lib/common.sh is sourced, not executed)
SCRIPT_NAMES: list[str] = ["build.sh", "launch.sh", "probe.sh", "format.sh"]


@pytest.mark.parametrize("script_name", SCRIPT_NAMES)
@pytest.mark.parametrize("help_flag", ["-h", "--help"])
def test_help_prints_usage(shell: ShellRunner, script_name: str, help_flag: str) -> None:
    """Every script prints its usage and exits successfully with -h/--help."""
    result_ = shell.run_script(script_name, [help_flag])
    assert result_.returncode == 0, result_.stderr
    assert result_.stdout.startswith(f"Usage: {script_name}")


@pytest.mark.parametrize("script_name", SCRIPT_NAMES)
def test_unknown_option_is_rejected(shell: ShellRunner, script_name: str) -> None:
    """Unknown options print the usage on stderr and fail."""
    result_ = shell.run_script(script_name, ["--no-such-option"])
    assert result_.returncode == 1
    assert "Unknown option: --no-such-option" in result_.stderr
    assert "Usage:" in result_.stderr


def test_launch_missing_config_file(shell: ShellRunner, tmp_path: Path) -> None:
    """launch.sh stops before anything else when the config file does not exist."""
    missing_config_ = tmp_path / "missing.yaml"
    result_ = shell.run_script("launch.sh", ["-c", str(missing_config_)])
    assert result_.returncode == 1
    assert f"Config file not found: {missing_config_}" in result_.stderr


def test_launch_config_option_requires_argument(shell: ShellRunner) -> None:
    """-c without a file name is an error, not a silent fallback to the default config."""
    result_ = shell.run_script("launch.sh", ["-c"])
    assert result_.returncode == 1
    assert "requires a file argument" in result_.stderr


def test_format_missing_clang_format(shell: ShellRunner) -> None:
    """format.sh explains how to install clang-format when it is not available."""
    result_ = shell.run_script("format.sh", [], extra_env={"CLANG_FORMAT": "no-such-clang-format"})
    assert result_.returncode == 1
    assert "sudo apt install clang-format" in result_.stderr
