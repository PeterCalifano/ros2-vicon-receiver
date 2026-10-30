"""Shared pytest fixtures for the repository tests.

Provides:
    - repository paths;
    - a runner for the functions of scripts/lib/common.sh and for the scripts in scripts/;
    - a factory of parameter files derived from config/vicon_client.yaml;
    - a local TCP listener standing in for the Vicon DataStream port;
    - the ROS 2 environment for integration tests (skipped when ROS or the build is missing).
"""

from __future__ import annotations

import os
import signal
import socket
import subprocess
import sys
import threading
import time
from collections.abc import Callable, Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pytest
import yaml

REPO_ROOT: Path = Path(__file__).resolve().parents[1]

# The Python tools are imported from their sources, so the unit tests run without a build
TOOLS_PYTHON_DIR: Path = REPO_ROOT / "vicon_receiver_tools"
if str(TOOLS_PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_PYTHON_DIR))
SCRIPTS_DIR: Path = REPO_ROOT / "scripts"
COMMON_SH: Path = SCRIPTS_DIR / "lib" / "common.sh"
CONFIG_FILE: Path = REPO_ROOT / "config" / "vicon_client.yaml"
FAKE_BIN_DIR: Path = REPO_ROOT / "tests" / "fixtures" / "fake_bin"
ROS_SETUP_FILE: Path = Path("/opt/ros/jazzy/setup.bash")
WORKSPACE_SETUP_FILE: Path = REPO_ROOT / "install" / "setup.bash"
NODE_EXECUTABLE: Path = (
    REPO_ROOT / "install" / "vicon_receiver" / "lib" / "vicon_receiver" / "vicon_client"
)
PROBE_EXECUTABLE: Path = NODE_EXECUTABLE.with_name("vicon_probe")

# Top-level key and parameters section of the parameters file
NODE_NAME: str = "vicon_client"
PARAMETERS_KEY: str = "ros__parameters"

# Default timeout of a single shell/script invocation [s]
DEFAULT_TIMEOUT_S: float = 30.0


# -------------------------------------------------------------------------------------------------
# Shell helpers runner
# -------------------------------------------------------------------------------------------------
@dataclass
class ShellRunner:
    """Run bash snippets with scripts/lib/common.sh sourced, or the scripts in scripts/.

    Attributes:
        base_env: Environment used for every invocation (extended per call).
    """

    base_env: dict[str, str]

    def run_common(
        self,
        snippet: str,
        extra_env: dict[str, str] | None = None,
        use_fake_bin: bool = False,
        timeout_s: float = DEFAULT_TIMEOUT_S,
    ) -> subprocess.CompletedProcess[str]:
        """Run a bash snippet after sourcing common.sh with the scripts' strict mode.

        Args:
            snippet: Bash code calling the common.sh functions.
            extra_env: Variables added to the environment of this call.
            use_fake_bin: Put tests/fixtures/fake_bin first in PATH (fake 'ip' and 'timeout').
            timeout_s: Maximum run time of the snippet.

        Returns:
            The completed process, with stdout and stderr captured as text.
        """
        script_ = f'set -Eeuo pipefail\nsource "{COMMON_SH}"\n{snippet}\n'
        return subprocess.run(
            ["bash", "-c", script_],
            env=self._make_env(extra_env, use_fake_bin),
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )

    def run_script(
        self,
        script_name: str,
        args: list[str],
        extra_env: dict[str, str] | None = None,
        use_fake_bin: bool = False,
        timeout_s: float = DEFAULT_TIMEOUT_S,
    ) -> subprocess.CompletedProcess[str]:
        """Run one of the scripts in scripts/ and capture its output.

        Args:
            script_name: File name in scripts/, e.g. "launch.sh".
            args: Command-line arguments.
            extra_env: Variables added to the environment of this call.
            use_fake_bin: Put tests/fixtures/fake_bin first in PATH.
            timeout_s: Maximum run time of the script.

        Returns:
            The completed process, with stdout and stderr captured as text.
        """
        return subprocess.run(
            [str(SCRIPTS_DIR / script_name), *args],
            env=self._make_env(extra_env, use_fake_bin),
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )

    def _make_env(self, extra_env: dict[str, str] | None, use_fake_bin: bool) -> dict[str, str]:
        """Build the environment of one invocation."""
        env_ = dict(self.base_env)
        if extra_env is not None:
            env_.update(extra_env)
        if use_fake_bin:
            env_["PATH"] = f"{FAKE_BIN_DIR}{os.pathsep}{env_['PATH']}"
        return env_


@pytest.fixture
def shell() -> ShellRunner:
    """Runner for common.sh functions and scripts, in the current environment."""
    return ShellRunner(base_env=dict(os.environ))


# -------------------------------------------------------------------------------------------------
# Parameter files
# -------------------------------------------------------------------------------------------------
def Load_default_parameters() -> dict[str, Any]:
    """Return the parameters section of config/vicon_client.yaml."""
    with CONFIG_FILE.open("r", encoding="utf-8") as stream_:
        document_ = yaml.safe_load(stream_)
    return dict(document_[NODE_NAME][PARAMETERS_KEY])


ConfigFactory = Callable[..., Path]


@pytest.fixture
def make_config(tmp_path: Path) -> ConfigFactory:
    """Factory writing parameter files derived from config/vicon_client.yaml.

    Usage: ``make_config(overrides={"hostname": "127.0.0.1:9000"}, remove=["buffer_size"])``.
    Values in ``overrides`` replace (or add) parameters; names in ``remove`` are dropped.
    """
    counter_ = 0

    def _Make_config(
        overrides: dict[str, Any] | None = None,
        remove: list[str] | None = None,
    ) -> Path:
        nonlocal counter_
        parameters_ = Load_default_parameters()
        if overrides is not None:
            parameters_.update(overrides)
        for name_ in remove or []:
            parameters_.pop(name_, None)

        counter_ += 1
        path_ = tmp_path / f"vicon_client_{counter_}.yaml"
        document_ = {NODE_NAME: {PARAMETERS_KEY: parameters_}}
        path_.write_text(yaml.safe_dump(document_, sort_keys=False), encoding="utf-8")
        return path_

    return _Make_config


# -------------------------------------------------------------------------------------------------
# Local TCP listener
# -------------------------------------------------------------------------------------------------
@dataclass
class TcpListener:
    """A local TCP server that accepts and immediately closes connections.

    Attributes:
        host: Address the server listens on.
        port: Port the server listens on (chosen by the OS).
    """

    host: str
    port: int

    @property
    def host_spec(self) -> str:
        """The "host:port" string used in the 'hostname' parameter."""
        return f"{self.host}:{self.port}"


@pytest.fixture
def tcp_listener() -> Iterator[TcpListener]:
    """Start a TCP listener on 127.0.0.1 standing in for the DataStream port."""
    server_socket_ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket_.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket_.bind(("127.0.0.1", 0))
    server_socket_.listen()
    server_socket_.settimeout(0.2)
    stop_event_ = threading.Event()

    def _Accept_loop() -> None:
        while not stop_event_.is_set():
            try:
                connection_, _ = server_socket_.accept()
            except (TimeoutError, OSError):
                continue
            connection_.close()

    thread_ = threading.Thread(target=_Accept_loop, daemon=True)
    thread_.start()

    host_, port_ = server_socket_.getsockname()
    yield TcpListener(host=host_, port=port_)

    stop_event_.set()
    thread_.join(timeout=2.0)
    server_socket_.close()


# -------------------------------------------------------------------------------------------------
# ROS 2 environment (integration tests)
# -------------------------------------------------------------------------------------------------
def Load_ros_environment() -> dict[str, str]:
    """Return the environment obtained by sourcing ROS 2 and the workspace overlay."""
    command_ = (
        "set +u; "
        f'source "{ROS_SETUP_FILE}"; '
        f'source "{WORKSPACE_SETUP_FILE}"; '
        "env -0"
    )
    output_ = subprocess.run(
        ["bash", "-c", command_],
        capture_output=True,
        check=True,
        timeout=DEFAULT_TIMEOUT_S,
    ).stdout

    environment_: dict[str, str] = {}
    for entry_ in output_.split(b"\0"):
        if b"=" not in entry_:
            continue
        name_, value_ = entry_.split(b"=", 1)
        environment_[name_.decode()] = value_.decode()
    return environment_


@pytest.fixture(scope="session")
def ros_env() -> dict[str, str]:
    """ROS 2 + workspace environment; skips the test if ROS or the build is missing."""
    if not ROS_SETUP_FILE.is_file():
        pytest.skip(f"ROS 2 not installed ({ROS_SETUP_FILE} missing)")
    if not NODE_EXECUTABLE.is_file():
        pytest.skip(f"workspace not built ({NODE_EXECUTABLE} missing): run scripts/build.sh")

    environment_ = Load_ros_environment()
    # Keep test traffic on this machine, away from other ROS 2 systems on the network
    environment_["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    return environment_


@pytest.fixture
def ros_shell(ros_env: dict[str, str]) -> ShellRunner:
    """Runner for scripts, in the ROS 2 + workspace environment."""
    return ShellRunner(base_env=dict(ros_env))


# -------------------------------------------------------------------------------------------------
# Long-running processes (the node runs until stopped)
# -------------------------------------------------------------------------------------------------
@dataclass
class ProcessOutcome:
    """Result of running a long-lived process until an expected output line.

    Attributes:
        found: Whether the expected text appeared before the timeout or the process exit.
        output: Combined stdout and stderr collected so far.
        returncode: Exit code if the process ended on its own, None if it was stopped by us.
    """

    found: bool
    output: str
    returncode: int | None


def Run_until_output(
    command: list[str],
    env: dict[str, str],
    expected_text: str,
    timeout_s: float = 20.0,
) -> ProcessOutcome:
    """Run a command until ``expected_text`` appears in its output, then stop it.

    The process runs in its own session; on stop, the whole process group receives SIGINT (like
    Ctrl+C) and, if still alive after a grace period, SIGKILL. This also stops the children of
    wrappers such as launch.sh / ros2 run.

    Args:
        command: Command and arguments.
        env: Environment of the process.
        expected_text: Text to wait for in the combined stdout/stderr.
        timeout_s: Maximum time to wait for the text.

    Returns:
        Whether the text was found, the collected output and the exit code (if it exited).
    """
    process_ = subprocess.Popen(
        command,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    lines_: list[str] = []
    found_event_ = threading.Event()

    def _Collect_output() -> None:
        assert process_.stdout is not None
        for line_ in process_.stdout:
            lines_.append(line_)
            if expected_text in "".join(lines_):
                found_event_.set()

    reader_ = threading.Thread(target=_Collect_output, daemon=True)
    reader_.start()

    # Wait for the text, or for the process to end by itself
    deadline_ = time.monotonic() + timeout_s
    while time.monotonic() < deadline_ and not found_event_.is_set():
        if process_.poll() is not None:
            break
        found_event_.wait(timeout=0.1)

    returncode_ = process_.poll()
    if returncode_ is None:
        Stop_process_group(process_)
    reader_.join(timeout=5.0)

    return ProcessOutcome(
        found=expected_text in "".join(lines_),
        output="".join(lines_),
        returncode=returncode_,
    )


def Stop_process_group(process: subprocess.Popen[str], grace_period_s: float = 8.0) -> None:
    """Stop a process started with start_new_session=True, together with its children."""
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=grace_period_s)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5.0)
    except ProcessLookupError:
        pass
