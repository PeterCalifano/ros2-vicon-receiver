"""Integration test of the error path of the G1 joint-state adapter (scripts/g1_joint_states.py).

The unitree_hg messages are not part of this repository. A stand-in package is enough here: the
adapter creates its publisher first, so an invalid topic stops it before LowState is used.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest
import yaml
from conftest import DEFAULT_TIMEOUT_S, REPO_ROOT, SCRIPTS_DIR

pytestmark = pytest.mark.integration


def test_invalid_topic_is_reported_without_a_traceback(
    ros_env: dict[str, str], tmp_path: Path
) -> None:
    """A topic name that ROS rejects ends in an error message and exit status 1."""
    package_ = tmp_path / "unitree_hg" / "msg"
    package_.mkdir(parents=True)
    (tmp_path / "unitree_hg" / "__init__.py").write_text("", encoding="utf-8")
    (package_ / "__init__.py").write_text("class LowState:\n    pass\n", encoding="utf-8")

    document_ = yaml.safe_load(
        (REPO_ROOT / "config" / "g1_joint_states.yaml").read_text(encoding="utf-8")
    )
    document_["g1_joint_states"]["joint_states_topic"] = "not a valid topic"
    config_ = tmp_path / "g1_joint_states.yaml"
    config_.write_text(yaml.safe_dump(document_), encoding="utf-8")

    environment_ = dict(ros_env, PYTHONPATH=f"{tmp_path}:{ros_env['PYTHONPATH']}")
    result_ = subprocess.run(
        [str(SCRIPTS_DIR / "g1_joint_states.py"), "-c", str(config_)],
        env=environment_,
        capture_output=True,
        text=True,
        timeout=DEFAULT_TIMEOUT_S,
        check=False,
    )
    assert result_.returncode == 1, result_.stdout + result_.stderr
    assert "invalid topic" in result_.stderr
    assert "Traceback" not in result_.stderr
