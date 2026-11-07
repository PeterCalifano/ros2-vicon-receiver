#!/usr/bin/env python3
"""estimate_mount.py - Estimate where the Vicon markers sit on the G1, for the live viewer.

Computes the pose of the mount link (torso_link) in the Vicon body frame from a probe recording
(scripts/probe.sh), and with --write saves it to the mount file used by
scripts/start_visualizer.sh. This script is the adapter between our set-up and the generic method
of vicon_receiver_tools/mount_estimate.py: it reads the set-up facts of config/viewer.yaml
(mount_estimate: left/right marker pairs, markers on other links) and turns marker names into the
indices and surfaces the method takes.

Usage:
    scripts/estimate_mount.py -r RECORDING.json [-c VIEWER_CONFIG] [--write]

Runs in the conda environment of environment.yml (trimesh, yourdfpy): started with another
interpreter, it restarts itself with the Python of that environment.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT: Path = Path(__file__).resolve().parents[1]
ENVIRONMENT_FILE: Path = REPO_ROOT / "environment.yml"
TOOLS_DIR: Path = REPO_ROOT / "vicon_receiver_tools"
DEFAULT_VIEWER_CONFIG: Path = REPO_ROOT / "config" / "viewer.yaml"
SCRIPT_NAME: str = Path(__file__).name

USAGE: str = f"""Usage: {SCRIPT_NAME} -r RECORDING.json [options]

Estimate the pose of the robot mount link in the Vicon body frame from a probe
recording: link upright, heading from the left/right marker pairs of the viewer
config, position fitted to the robot meshes. For the viewer, not a calibration.

Options:
  -r, --recording FILE  Probe recording (scripts/probe.sh), with markers
  -c, --config FILE     Viewer config (default: config/viewer.yaml)
      --write           Save the result to the mount file of the viewer config
  -h, --help            Show this help and exit
"""


def Fail(message: str) -> int:
    """Print an error like the shell scripts do and return the exit status 1."""
    print(f"[{SCRIPT_NAME}] ERROR: {message}", file=sys.stderr)
    return 1


def Tools_env_name() -> str:
    """Name of the conda environment of the tools, from environment.yml."""
    match_ = re.search(r"^name:\s*(\S+)", ENVIRONMENT_FILE.read_text(encoding="utf-8"), re.MULTILINE)
    return match_.group(1) if match_ else ""


def Tools_env_prefix(name: str) -> Path | None:
    """Directory of a conda environment, or None if conda or the environment is missing."""
    conda_ = os.environ.get("CONDA_EXE") or shutil.which("conda")
    if not conda_:
        return None
    listing_ = subprocess.run([conda_, "env", "list"], capture_output=True, text=True, check=False).stdout
    for line_ in listing_.splitlines():
        fields_ = line_.split()
        if fields_ and fields_[0] == name:
            return Path(fields_[-1])
    return None


def Parse_arguments(argv: list[str]) -> dict[str, object] | int:
    """Options of the command line, or an exit status (help shown, or an error)."""
    options_: dict[str, object] = {
        "recording": None,
        "config": DEFAULT_VIEWER_CONFIG,
        "write": False,
    }
    index_ = 0
    while index_ < len(argv):
        argument_ = argv[index_]
        if argument_ in ("-h", "--help"):
            print(USAGE, end="")
            return 0
        if argument_ in ("-r", "--recording", "-c", "--config"):
            if index_ + 1 >= len(argv):
                return Fail(f"Option {argument_} requires a file argument.")
            key_ = "recording" if argument_ in ("-r", "--recording") else "config"
            options_[key_] = Path(argv[index_ + 1])
            index_ += 2
            continue
        if argument_ == "--write":
            options_["write"] = True
            index_ += 1
            continue
        print(USAGE, end="", file=sys.stderr)
        return Fail(f"Unknown option: {argument_}")
    return options_


def Main(argv: list[str]) -> int:
    """Check the inputs, then run the estimate in the conda environment of the tools."""
    options_ = Parse_arguments(argv)
    if isinstance(options_, int):
        return options_

    recording_ = options_["recording"]
    config_ = options_["config"]
    if recording_ is None:
        print(USAGE, end="", file=sys.stderr)
        return Fail("A probe recording is required (-r).")
    assert isinstance(recording_, Path) and isinstance(config_, Path)
    for path_ in (recording_, config_):
        if not path_.is_file():
            return Fail(f"File not found: {path_}")

    # trimesh and yourdfpy live in the conda environment of the tools
    env_name_ = Tools_env_name()
    if Path(sys.prefix).name != env_name_:
        prefix_ = Tools_env_prefix(env_name_)
        if prefix_ is None or not (prefix_ / "bin" / "python").exists():
            return Fail(f"Conda environment '{env_name_}' not found. Create it with: conda env create -f {ENVIRONMENT_FILE}")
        env_python_ = str(prefix_ / "bin" / "python")
        os.execv(env_python_, [env_python_, str(Path(__file__).resolve()), *argv])

    # Tools from their sources, as the shell scripts do (works before the build)
    sys.path.insert(0, str(TOOLS_DIR))
    return Estimate(recording_, config_, bool(options_["write"]))


def Estimate(recording: Path, viewer_config: Path, write: bool) -> int:
    """Map the set-up facts of the viewer config onto the generic mount estimate and run it."""
    import json

    from vicon_receiver_tools import mount_estimate
    from vicon_receiver_tools.config import ConfigError
    from vicon_receiver_tools.robot_model import Link_mesh, Load_urdf, RobotModelError, Rigid_mesh
    from vicon_receiver_tools.viewer_config import Load_viewer_config

    try:
        config_ = Load_viewer_config(viewer_config)
        settings_ = config_.mount_estimate
        recording_data_ = json.loads(recording.read_text(encoding="utf-8"))
        names_, body_points_ = mount_estimate.Marker_constellation(recording_data_, config_.robot.subject)

        # Marker names of the config -> indices in the recording
        index_of_ = {name_: index_ for index_, name_ in enumerate(names_)}
        configured_ = [name_ for pair_ in settings_.lateral_marker_pairs for name_ in (pair_.left, pair_.right)]
        missing_ = [name_ for name_ in configured_ + list(settings_.marker_links) if name_ not in index_of_]
        if missing_:
            return Fail(f"markers of the viewer config not visible in the recording: {', '.join(missing_)}")
        pairs_ = [(index_of_[pair_.left], index_of_[pair_.right]) for pair_ in settings_.lateral_marker_pairs]

        # Surface of each marker: its configured link, else the mount link and the links fixed to it
        model_dir_ = REPO_ROOT / config_.robot.directory
        urdf_ = Load_urdf(model_dir_, config_.robot.urdf, load_meshes=False)
        mount_link_ = config_.robot.mount_link
        surfaces_by_link_ = {
            None: mount_estimate.SampledMeshSurface.From_mesh(Rigid_mesh(urdf_, mount_link_, model_dir_))
        }
        for link_ in sorted(set(settings_.marker_links.values())):
            surfaces_by_link_[link_] = mount_estimate.SampledMeshSurface.From_mesh(
                Link_mesh(urdf_, link_, mount_link_, model_dir_)
            )
        surfaces_ = [surfaces_by_link_[settings_.marker_links.get(name_)] for name_ in names_]

        estimate_ = mount_estimate.Estimate_upright_mount(body_points_, pairs_, surfaces_, settings_.surface_offset_m)
    except (OSError, json.JSONDecodeError, mount_estimate.MountEstimateError, RobotModelError, ConfigError, ValueError) as error_:
        return Fail(f"mount estimate failed: {error_}")

    print(mount_estimate.Report(estimate_, names_, pairs_, mount_link_, config_.robot.subject))
    if write:
        mount_path_ = REPO_ROOT / config_.robot.mount_file
        resolved_ = recording.resolve()
        shown_ = resolved_.relative_to(REPO_ROOT) if resolved_.is_relative_to(REPO_ROOT) else resolved_
        mount_estimate.Write_mount_file(mount_path_, mount_link_, config_.robot.subject, estimate_, str(shown_))
        print(f"written: {mount_path_}")
    return 0

if __name__ == "__main__":
    sys.exit(Main(sys.argv[1:]))
