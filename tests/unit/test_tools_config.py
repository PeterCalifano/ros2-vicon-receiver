"""Unit tests of vicon_receiver_tools.config (access to the node parameters file)."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import pytest

from conftest import CONFIG_FILE
from vicon_receiver_tools.config import (
    ConfigError,
    Format_value,
    Load_node_parameters,
    Main,
    Read_parameter,
)


# -------------------------------------------------------------------------------------------------
# Load_node_parameters / Read_parameter
# -------------------------------------------------------------------------------------------------
def test_repository_config_has_all_node_parameters() -> None:
    """config/vicon_client.yaml defines every parameter the node declares."""
    parameters_ = Load_node_parameters(CONFIG_FILE)
    expected_names_ = {
        "hostname",
        "buffer_size",
        "namespace",
        "world_frame",
        "vicon_frame",
        "map_xyz",
        "map_rpy",
        "map_rpy_in_degrees",
    }
    assert set(parameters_) == expected_names_


def test_read_parameter_returns_typed_values(make_config: Callable[..., Path]) -> None:
    """Values keep their YAML types (the node rejects e.g. integers for doubles)."""
    config_ = make_config(overrides={"hostname": "10.1.2.3:900", "map_xyz": [1.0, 2.0, 3.0]})
    assert Read_parameter(config_, "hostname") == "10.1.2.3:900"
    assert Read_parameter(config_, "buffer_size") == 200
    assert Read_parameter(config_, "map_xyz") == [1.0, 2.0, 3.0]
    assert Read_parameter(config_, "map_rpy_in_degrees") is False


def test_read_parameter_missing_key(make_config: Callable[..., Path]) -> None:
    """A missing parameter is reported with its full path in the file."""
    config_ = make_config(remove=["hostname"])
    with pytest.raises(ConfigError, match=r"vicon_client\.ros__parameters\.hostname not found"):
        Read_parameter(config_, "hostname")


def test_missing_file_cannot_be_read(tmp_path: Path) -> None:
    """A non-existent file is reported as unreadable, with the reason."""
    with pytest.raises(ConfigError, match="cannot read .*No such file or directory"):
        Load_node_parameters(tmp_path / "missing.yaml")


def test_malformed_yaml_cannot_be_parsed(tmp_path: Path) -> None:
    """Invalid YAML is reported as unparsable."""
    config_ = tmp_path / "broken.yaml"
    config_.write_text("vicon_client: [unclosed\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="cannot parse"):
        Load_node_parameters(config_)


@pytest.mark.parametrize(
    ("content", "missing_section"),
    [
        ("other_node:\n  ros__parameters:\n    hostname: '1.2.3.4'\n", "'vicon_client'"),
        ("- just\n- a list\n", "'vicon_client'"),
        ("", "'vicon_client'"),
        ("vicon_client:\n  hostname: '1.2.3.4'\n", "'vicon_client.ros__parameters'"),
    ],
)
def test_wrong_layout_names_the_missing_section(
    tmp_path: Path, content: str, missing_section: str
) -> None:
    """Files without <node>.ros__parameters name the section that is missing."""
    config_ = tmp_path / "layout.yaml"
    config_.write_text(content, encoding="utf-8")
    with pytest.raises(ConfigError, match=f"section {missing_section} not found"):
        Load_node_parameters(config_)


def test_other_node_name(tmp_path: Path) -> None:
    """The node name can be chosen (the scripts pass vicon_client explicitly)."""
    config_ = tmp_path / "other_node.yaml"
    config_.write_text("probe:\n  ros__parameters:\n    frames: 10\n", encoding="utf-8")
    assert Read_parameter(config_, "frames", node_name="probe") == 10


# -------------------------------------------------------------------------------------------------
# Format_value
# -------------------------------------------------------------------------------------------------
@pytest.mark.parametrize(
    ("value", "expected_text"),
    [
        ("192.168.10.1", "192.168.10.1"),
        (200, "200"),
        (0.5, "0.5"),
        (True, "true"),
        (False, "false"),
        ([0.0, 0.0, 0.0], "[0.0, 0.0, 0.0]"),
    ],
)
def test_format_value(value: object, expected_text: str) -> None:
    """Values are printed the way they are written in YAML."""
    assert Format_value(value) == expected_text


# -------------------------------------------------------------------------------------------------
# Command line
# -------------------------------------------------------------------------------------------------
def test_main_prints_value(capsys: pytest.CaptureFixture[str]) -> None:
    """The command prints only the value on stdout and exits with 0."""
    exit_code_ = Main([str(CONFIG_FILE), "hostname"])
    captured_ = capsys.readouterr()
    assert exit_code_ == 0
    assert captured_.out == "192.168.10.1\n"
    assert captured_.err == ""


def test_main_reports_error(
    capsys: pytest.CaptureFixture[str], make_config: Callable[..., Path]
) -> None:
    """On error the command prints only the message on stderr and exits with 1."""
    config_ = make_config(remove=["namespace"])
    exit_code_ = Main([str(config_), "namespace"])
    captured_ = capsys.readouterr()
    assert exit_code_ == 1
    assert captured_.out == ""
    assert "vicon_client.ros__parameters.namespace not found" in captured_.err
