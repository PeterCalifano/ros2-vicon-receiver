"""Unit tests of the command-line interface of the scripts in scripts/ and of tests/run_tests.sh.

Only argument handling, the usage text and early validation are tested here: nothing is installed,
built or launched. The end-to-end behaviour of launch.sh is covered by tests/integration.
"""

from __future__ import annotations

import os
import re
import shutil
from pathlib import Path

import pytest
from conftest import REPO_ROOT, ROS_SETUP_FILE, SCRIPTS_DIR, ShellRunner

# Fake python3 for tests/run_tests.sh: every call succeeds, as colcon does when no test ran;
# "colcon test" leaves a result file if FAKE_RESULT_FILE is set
FAKE_COLCON_PYTHON: str = """#!/usr/bin/env bash
if [[ "$*" == "-m colcon test "* && -n "${FAKE_RESULT_FILE:-}" ]]; then
    mkdir -p "$(dirname "${FAKE_RESULT_FILE}")"
    touch "${FAKE_RESULT_FILE}"
fi
"""

# Scripts with a command-line interface, relative to scripts/ (lib/*.sh are sourced, not executed;
# source_ros_env.sh is sourced and has no options)
SCRIPT_PATHS: list[str] = [
    *sorted(
        path_.name
        for path_ in [*SCRIPTS_DIR.glob("*.sh"), *SCRIPTS_DIR.glob("*.py")]
        if path_.name != "source_ros_env.sh"
    ),
    "../tests/run_tests.sh",
]

# Option line of a usage: "  -c, --config FILE  ..." or "      --no-check  ..."
OPTION_LINE = re.compile(r"^ {2,6}(?:(-\w), )?(--[a-z][\w-]*)")


def Script_name(script_path: str) -> str:
    """File name of a script, as its usage shows it."""
    return Path(script_path).name


def Help_text(shell: ShellRunner, script_path: str) -> str:
    """Output of --help, which must succeed."""
    result_ = shell.run_script(script_path, ["--help"])
    assert result_.returncode == 0, result_.stderr
    return result_.stdout


@pytest.mark.parametrize("script_path", SCRIPT_PATHS)
@pytest.mark.parametrize("help_flag", ["-h", "--help"])
def test_help_prints_usage(shell: ShellRunner, script_path: str, help_flag: str) -> None:
    """Every script prints its usage and exits successfully with -h/--help."""
    result_ = shell.run_script(script_path, [help_flag])
    assert result_.returncode == 0, result_.stderr
    assert result_.stdout.startswith(f"Usage: {Script_name(script_path)}")


@pytest.mark.parametrize("script_path", SCRIPT_PATHS)
def test_help_has_the_common_sections(shell: ShellRunner, script_path: str) -> None:
    """Every usage has the same layout: synopsis, description, Options with -h, Examples."""
    help_ = Help_text(shell, script_path)
    assert "\nOptions:\n" in help_
    assert "  -h, --help " in help_
    assert "\nExamples:\n" in help_
    # Examples call the script by its path from the repository root
    examples_ = help_.split("\nExamples:\n", 1)[1]
    assert f"/{Script_name(script_path)}" in examples_


@pytest.mark.parametrize("script_path", SCRIPT_PATHS)
def test_synopsis_lists_every_option(shell: ShellRunner, script_path: str) -> None:
    """Each option of the Options section appears in the synopsis (short or long form)."""
    help_ = Help_text(shell, script_path)
    synopsis_ = help_.split("\n\n", 1)[0]
    options_section_ = help_.split("\nOptions:\n", 1)[1].split("\n\n", 1)[0]
    options_ = [OPTION_LINE.match(line_) for line_ in options_section_.splitlines()]
    options_ = [match_ for match_ in options_ if match_ is not None]
    assert options_, "no option found in the Options section"
    for match_ in options_:
        short_, long_ = match_.groups()
        assert long_ in synopsis_ or (short_ is not None and short_ in synopsis_), (
            f"{long_} missing from the synopsis: {synopsis_!r}"
        )


@pytest.mark.parametrize("script_path", SCRIPT_PATHS)
def test_unknown_option_is_rejected(shell: ShellRunner, script_path: str) -> None:
    """Unknown options print the usage on stderr and fail."""
    result_ = shell.run_script(script_path, ["--no-such-option"])
    assert result_.returncode == 1
    assert "Unknown option: --no-such-option" in result_.stderr
    assert "Usage:" in result_.stderr


@pytest.mark.parametrize(
    ("script_path", "option"),
    [
        ("launch.sh", "-c"),
        ("probe.sh", "--host"),
        ("probe.sh", "-n"),
        ("record.sh", "-t"),
        ("export.sh", "--format"),
        ("start_visualizer.sh", "--config"),
        ("fetch_g1_model.sh", "-c"),
        ("estimate_mount.py", "-r"),
        ("g1_joint_states.py", "-c"),
    ],
)
def test_option_without_value_is_rejected(
    shell: ShellRunner, script_path: str, option: str
) -> None:
    """An option given without its value is an error, not a silent fallback to a default."""
    result_ = shell.run_script(script_path, [option])
    assert result_.returncode == 1
    assert f"Option {option} requires a value." in result_.stderr


def test_launch_missing_config_file(shell: ShellRunner, tmp_path: Path) -> None:
    """launch.sh stops before anything else when the config file does not exist."""
    missing_config_ = tmp_path / "missing.yaml"
    result_ = shell.run_script("launch.sh", ["-c", str(missing_config_)])
    assert result_.returncode == 1
    assert f"Config file not found: {missing_config_}" in result_.stderr


def test_export_requires_session_and_format(shell: ShellRunner) -> None:
    """export.sh exports nothing by default: the format is required."""
    result_ = shell.run_script("export.sh", ["recordings/some_session"])
    assert result_.returncode == 1
    assert "A session directory and --format are required." in result_.stderr
    assert "Usage:" in result_.stderr


def test_estimate_mount_requires_a_recording(shell: ShellRunner) -> None:
    """estimate_mount.py without -r says that the recording is required."""
    result_ = shell.run_script("estimate_mount.py", [])
    assert result_.returncode == 1
    assert "A probe recording is required (-r)." in result_.stderr


def test_estimate_mount_stops_if_the_restart_missed_the_environment(
    shell: ShellRunner, tmp_path: Path
) -> None:
    """After one restart in the conda environment it stops instead of restarting for ever."""
    recording_ = tmp_path / "recording.json"
    recording_.write_text("{}", encoding="utf-8")
    result_ = shell.run_script(
        "estimate_mount.py",
        ["-r", str(recording_)],
        extra_env={"PATH": "/usr/bin:/bin", "VICON_ESTIMATE_MOUNT_RESTARTED": "1"},
    )
    assert result_.returncode == 1
    assert "which is not the environment" in result_.stderr


def test_run_tests_rejects_cpp_and_python_together(shell: ShellRunner) -> None:
    """--cpp and --python would run nothing: refused instead of reporting success."""
    result_ = shell.run_script("../tests/run_tests.sh", ["--cpp", "--python"])
    assert result_.returncode == 1
    assert "--cpp and --python exclude each other." in result_.stderr


@pytest.mark.parametrize(("tests_ran", "expected_status"), [(False, 1), (True, 0)])
def test_run_tests_fails_when_no_cpp_test_ran(
    shell: ShellRunner, tmp_path: Path, tests_ran: bool, expected_status: int
) -> None:
    """colcon says "0 tests" and succeeds if none was compiled: that must not count as a pass."""
    if not ROS_SETUP_FILE.is_file():
        pytest.skip(f"ROS 2 not installed ({ROS_SETUP_FILE} missing)")

    # A copy of the scripts: run_tests.sh removes the test results of the workspace it belongs to
    workspace_ = tmp_path / "workspace"
    shutil.copytree(SCRIPTS_DIR / "lib", workspace_ / "scripts" / "lib")
    (workspace_ / "tests").mkdir()
    run_tests_ = Path(shutil.copy2(REPO_ROOT / "tests" / "run_tests.sh", workspace_ / "tests"))

    # Stands in for "python3 -m colcon ...", which always succeeds
    fake_bin_ = tmp_path / "bin"
    fake_bin_.mkdir()
    (fake_bin_ / "python3").write_text(FAKE_COLCON_PYTHON, encoding="utf-8")
    (fake_bin_ / "python3").chmod(0o755)
    environment_ = {"PATH": f"{fake_bin_}{os.pathsep}{os.environ['PATH']}"}
    if tests_ran:
        environment_["FAKE_RESULT_FILE"] = str(
            workspace_ / "build" / "vicon_receiver" / "test_results" / "a" / "a.gtest.xml"
        )

    result_ = shell.run_script(str(run_tests_), ["--cpp", "--no-build"], extra_env=environment_)

    assert result_.returncode == expected_status, result_.stdout + result_.stderr
    assert ("No C++ test ran" in result_.stderr) == (not tests_ran)


def test_format_missing_clang_format(shell: ShellRunner) -> None:
    """format.sh explains how to install clang-format when it is not available."""
    result_ = shell.run_script("format.sh", [], extra_env={"CLANG_FORMAT": "no-such-clang-format"})
    assert result_.returncode == 1
    assert "sudo apt install clang-format" in result_.stderr
