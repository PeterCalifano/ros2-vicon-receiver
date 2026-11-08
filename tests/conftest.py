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

import numpy as np
import pytest
import yaml

REPO_ROOT: Path = Path(__file__).resolve().parents[1]

# The Python tools are imported from their sources, so the unit tests run without a build
TOOLS_PYTHON_DIR: Path = REPO_ROOT / "vicon_receiver_tools"
if str(TOOLS_PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_PYTHON_DIR))
SCRIPTS_DIR: Path = REPO_ROOT / "scripts"
COMMON_SH: Path = SCRIPTS_DIR / "lib" / "common.sh"
ROS_ENV_SH: Path = SCRIPTS_DIR / "lib" / "ros_env.sh"
CONFIG_FILE: Path = REPO_ROOT / "config" / "vicon_client.yaml"
FAKE_BIN_DIR: Path = REPO_ROOT / "tests" / "fixtures" / "fake_bin"


def Ros_env_variable(name: str) -> str:
    """Value of a variable defined by scripts/lib/ros_env.sh (the ROS distribution and setup files
    are written only there)."""
    return subprocess.run(
        ["bash", "-c", f'source "{ROS_ENV_SH}"; printf "%s" "${{{name}}}"'],
        capture_output=True,
        text=True,
        check=True,
    ).stdout


def Tools_env_python() -> str:
    """Python of the conda environment of the tools (environment.yml), or "" if it is missing."""
    return subprocess.run(
        ["bash", "-c", f'source "{COMMON_SH}"; tools_env_bin python'],
        capture_output=True,
        text=True,
        check=False,
    ).stdout.strip()


ROS_SETUP_FILE: Path = Path(Ros_env_variable("VICON_ROS_SETUP_FILE"))
WORKSPACE_SETUP_FILE: Path = Path(Ros_env_variable("VICON_WORKSPACE_SETUP_FILE"))
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
# Live viewer scene
# -------------------------------------------------------------------------------------------------
VIEWER_CONFIG: Path = REPO_ROOT / "config" / "viewer.yaml"

# Tiny URDF: base -> torso (revolute about z, 1 m up), camera fixed 0.1 m ahead of the torso
JOINTED_URDF: str = """<?xml version="1.0"?>
<robot name="jointed">
  <link name="base"/>
  <link name="torso"/>
  <link name="camera"/>
  <joint name="waist" type="revolute">
    <parent link="base"/><child link="torso"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-2" upper="2" effort="1" velocity="1"/>
  </joint>
  <joint name="camera_mount" type="fixed">
    <parent link="torso"/><child link="camera"/><origin xyz="0.1 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


def Write_viewer_config(tmp_path: Path, edit: Callable[[dict[str, Any]], None]) -> Path:
    """Copy of config/viewer.yaml, changed by a function that edits the parsed document."""
    document_ = yaml.safe_load(VIEWER_CONFIG.read_text(encoding="utf-8"))
    edit(document_)
    path_ = tmp_path / "viewer.yaml"
    path_.write_text(yaml.safe_dump(document_, sort_keys=False), encoding="utf-8")
    return path_


def Subject_at_origin(name: str, ros_name: str) -> Any:
    """ViewerSubject with one visible segment at the origin and one visible marker."""
    from vicon_receiver_tools.viewer import ViewerMarker, ViewerSegment, ViewerSubject

    return ViewerSubject(
        name=name,
        ros_name=ros_name,
        quality=1.0,
        segments=[
            ViewerSegment(name="root", position=np.zeros(3), wxyz=np.array([1.0, 0.0, 0.0, 0.0]), occluded=False)
        ],
        markers=[ViewerMarker(name="m1", position=np.zeros(3), camera_count=4, occluded=False)],
    )


@dataclass
class ViewerScene:
    """Viewer on a local viser server.

    Attributes:
        config: Its ViewerConfig.
        viewer: The ViserLiveViewer.
    """

    config: Any
    viewer: Any


@pytest.fixture
def jointed_robot_scene(tmp_path: Path) -> Iterator[ViewerScene]:
    """Viewer of JOINTED_URDF (identity mount) on a local server; skipped without viser/yourdfpy."""
    viser_ = pytest.importorskip("viser")
    pytest.importorskip("yourdfpy")
    from vicon_receiver_tools.viewer import ViserLiveViewer
    from vicon_receiver_tools.viewer_config import Load_viewer_config

    (tmp_path / "model").mkdir()
    (tmp_path / "model" / "jointed.urdf").write_text(JOINTED_URDF, encoding="utf-8")
    with socket.socket() as free_socket_:
        free_socket_.bind(("127.0.0.1", 0))
        port_ = free_socket_.getsockname()[1]

    def _Edit(document_: dict[str, Any]) -> None:
        document_["viewer"]["port"] = port_
        document_["robot"].update(
            directory="model", urdf="jointed.urdf", mount_link="torso", mount_file="no_mount.yaml"
        )
        document_["cameras"] = [dict(document_["cameras"][0], name="cam", frame="camera")]

    config_ = Load_viewer_config(Write_viewer_config(tmp_path, _Edit))
    server_ = viser_.ViserServer(host="127.0.0.1", port=port_, verbose=False)
    try:
        yield ViewerScene(config=config_, viewer=ViserLiveViewer(config_, tmp_path, server_))
    finally:
        server_.stop()


# -------------------------------------------------------------------------------------------------
# Child processes
# -------------------------------------------------------------------------------------------------
def Wait_or_kill(process: subprocess.Popen, timeout_s: float) -> None:
    """Wait for a child process; past timeout_s, kill it with its process group (if it leads one).

    Meant for finally blocks: a failing integration test must not leave ROS processes behind.
    """
    try:
        process.wait(timeout=timeout_s)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        if os.getpgid(process.pid) == process.pid:
            os.killpg(process.pid, signal.SIGKILL)
        else:
            process.kill()
    except ProcessLookupError:
        pass
    process.wait(timeout=DEFAULT_TIMEOUT_S)


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
    """Return the environment obtained by sourcing ROS 2 and the workspace overlay.

    The DDS configuration of the machine is applied as in the scripts (scripts/lib/ros_env.sh),
    so that the ROS 2 processes started by the tests see each other also on WSL.
    """
    command_ = (
        f'source "{ROS_ENV_SH}"; '
        'vicon_source_setup_file "${VICON_ROS_SETUP_FILE}"; '
        'vicon_source_setup_file "${VICON_WORKSPACE_SETUP_FILE}"; '
        "vicon_configure_dds >/dev/null; "
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
    # Keep test traffic on this machine, away from other ROS 2 systems on the network. With the
    # WSL profile (scripts/lib/ros_env.sh) traffic is already local, and LOCALHOST would override the
    # profile with redirected 127.0.0.1 peers: let the profile decide.
    if environment_.get("FASTRTPS_DEFAULT_PROFILES_FILE"):
        environment_["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "SYSTEM_DEFAULT"
    else:
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
