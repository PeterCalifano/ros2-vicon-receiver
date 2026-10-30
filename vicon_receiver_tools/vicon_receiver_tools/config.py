"""Access to the vicon_receiver parameters file (a ROS 2 --params-file YAML).

The file has the layout::

    vicon_client:
      ros__parameters:
        hostname: "192.168.10.1"
        ...

Command line (used by scripts/lib/common.sh)::

    python3 -m vicon_receiver_tools.config CONFIG_FILE PARAM_NAME [--node NODE_NAME]

prints the value of ``NODE_NAME.ros__parameters.PARAM_NAME`` and exits with 0, or prints the
problem on stderr and exits with 1.

Example::

    $ python3 -m vicon_receiver_tools.config config/vicon_client.yaml hostname
    192.168.10.1
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import yaml

# Name of the node, i.e. the top-level key of the parameters file
DEFAULT_NODE_NAME: str = "vicon_client"

# Key of the parameters section under the node name (fixed by ROS 2)
PARAMETERS_KEY: str = "ros__parameters"


class ConfigError(Exception):
    """The parameters file cannot be read or does not contain the requested entry."""


def Load_node_parameters(config_file: Path, node_name: str = DEFAULT_NODE_NAME) -> dict[str, Any]:
    """Return the parameters section of one node from a ROS 2 parameters file.

    Args:
        config_file: Path of the YAML parameters file.
        node_name: Top-level key of the node in the file.

    Returns:
        The mapping under ``node_name.ros__parameters``.

    Raises:
        ConfigError: If the file cannot be read or parsed, or has no parameters for the node.
    """
    try:
        with config_file.open("r", encoding="utf-8") as stream_:
            document_ = yaml.safe_load(stream_)
    except OSError as error_:
        raise ConfigError(f"cannot read {config_file}: {error_.strerror}") from error_
    except yaml.YAMLError as error_:
        raise ConfigError(f"cannot parse {config_file}: {error_}") from error_

    # Each level must be a mapping; anything else means the layout is wrong
    node_section_ = document_.get(node_name) if isinstance(document_, dict) else None
    if not isinstance(node_section_, dict):
        raise ConfigError(f"section '{node_name}' not found in {config_file}")

    parameters_ = node_section_.get(PARAMETERS_KEY)
    if not isinstance(parameters_, dict):
        raise ConfigError(f"section '{node_name}.{PARAMETERS_KEY}' not found in {config_file}")

    return parameters_


def Read_parameter(
    config_file: Path, param_name: str, node_name: str = DEFAULT_NODE_NAME
) -> Any:
    """Return the value of one parameter from a ROS 2 parameters file.

    Args:
        config_file: Path of the YAML parameters file.
        param_name: Name of the parameter under ``node_name.ros__parameters``.
        node_name: Top-level key of the node in the file.

    Raises:
        ConfigError: If the file is invalid or the parameter is missing.
    """
    parameters_ = Load_node_parameters(config_file, node_name)
    if param_name not in parameters_:
        raise ConfigError(f"{node_name}.{PARAMETERS_KEY}.{param_name} not found in {config_file}")
    return parameters_[param_name]


def Format_value(value: Any) -> str:
    """Format a parameter value as text for shell scripts.

    Booleans use the YAML spelling (true/false); lists and mappings use the YAML flow style
    (e.g. ``[0.0, 0.0, 0.0]``); any other value is converted with str().
    """
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, dict)):
        return yaml.safe_dump(value, default_flow_style=True).strip()
    return str(value)


def Main(argv: list[str] | None = None) -> int:
    """Command-line entry point: print one parameter of the parameters file.

    Returns:
        Process exit code: 0 on success, 1 if the parameter cannot be read.
    """
    parser_ = argparse.ArgumentParser(
        prog="python3 -m vicon_receiver_tools.config",
        description="Print the value of one parameter from a ROS 2 parameters file.",
    )
    parser_.add_argument("config_file", type=Path, help="YAML parameters file")
    parser_.add_argument("param_name", help="parameter name under <node>.ros__parameters")
    parser_.add_argument(
        "--node",
        default=DEFAULT_NODE_NAME,
        help=f"top-level node key in the file (default: {DEFAULT_NODE_NAME})",
    )
    arguments_ = parser_.parse_args(argv)

    try:
        value_ = Read_parameter(arguments_.config_file, arguments_.param_name, arguments_.node)
    except ConfigError as error_:
        print(error_, file=sys.stderr)
        return 1

    print(Format_value(value_))
    return 0


if __name__ == "__main__":
    sys.exit(Main())
