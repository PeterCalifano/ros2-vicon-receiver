"""Unit tests of scripts/lib/dds.sh (DDS configuration for WSL mirrored networking).

The fake 'ip' command in tests/fixtures/fake_bin reproduces the routes of a normal Linux machine
and of WSL with mirrored networking, so the detection runs the same on every machine.
"""

from __future__ import annotations

import os

import pytest

from conftest import DDS_SH, REPO_ROOT, ShellRunner

# "ip -4 route get 127.0.0.1" on a normal Linux machine
ROUTE_LOCAL: str = "local 127.0.0.1 dev lo table local src 127.0.0.1 uid 1000"
# The same on WSL with mirrored networking: localhost goes to Windows
ROUTE_WSL_MIRRORED: str = (
    "127.0.0.1 via 169.254.73.152 dev loopback0 table 127 src 127.0.0.1 uid 1000"
)

# "ip -4 addr show dev lo" with and without the address used by config/fastdds_wsl.xml
LO_WITH_WSL_ADDRESS: str = (
    "1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000\n"
    "    inet 127.0.0.1/8 scope host lo\n"
    "    inet 10.255.255.254/32 brd 10.255.255.254 scope global lo"
)
LO_WITHOUT_WSL_ADDRESS: str = (
    "1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000\n"
    "    inet 127.0.0.1/8 scope host lo"
)

PROFILE_FILE: str = str(REPO_ROOT / "config" / "fastdds_wsl.xml")


def Run_dds(
    shell: ShellRunner,
    snippet: str,
    route: str,
    lo_addresses: str = LO_WITH_WSL_ADDRESS,
    preset_profile: str | None = None,
):
    """Run a snippet with dds.sh sourced and the fake network state; return the process."""
    environment_ = {
        "FAKE_IP_ROUTE_OUTPUT": route,
        "FAKE_IP_LO_ADDR_OUTPUT": lo_addresses,
    }
    # Start from a clean state unless the test sets a profile on purpose
    unset_ = "unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE; "
    if preset_profile is not None:
        environment_["FASTRTPS_DEFAULT_PROFILES_FILE"] = preset_profile
        unset_ = ""
    script_ = f'{unset_}source "{DDS_SH}"; {snippet}'
    return shell.run_common(script_, extra_env=environment_, use_fake_bin=True)


# -------------------------------------------------------------------------------------------------
# Detection
# -------------------------------------------------------------------------------------------------
def test_local_route_is_not_redirected(shell: ShellRunner) -> None:
    """On a normal machine localhost stays on lo."""
    result_ = Run_dds(shell, "vicon_localhost_redirected && echo yes || echo no", ROUTE_LOCAL)
    assert result_.stdout.strip() == "no"


def test_wsl_mirrored_route_is_redirected(shell: ShellRunner) -> None:
    """WSL mirrored networking sends localhost to Windows via loopback0."""
    result_ = Run_dds(
        shell, "vicon_localhost_redirected && echo yes || echo no", ROUTE_WSL_MIRRORED
    )
    assert result_.stdout.strip() == "yes"


def test_unknown_route_is_not_redirected(shell: ShellRunner) -> None:
    """Without route information nothing is changed."""
    result_ = Run_dds(shell, "vicon_localhost_redirected && echo yes || echo no", "")
    assert result_.stdout.strip() == "no"


def test_profile_file_points_to_repository_config(shell: ShellRunner) -> None:
    """The profile is the one in config/, whatever the current directory."""
    result_ = Run_dds(shell, "cd /; vicon_dds_profile_file", ROUTE_LOCAL)
    assert result_.stdout.strip() == PROFILE_FILE
    assert os.path.isfile(PROFILE_FILE)


# -------------------------------------------------------------------------------------------------
# Configuration
# -------------------------------------------------------------------------------------------------
PRINT_PROFILES: str = (
    'vicon_configure_dds; '
    'echo "FASTRTPS=${FASTRTPS_DEFAULT_PROFILES_FILE:-unset}"; '
    'echo "FASTDDS=${FASTDDS_DEFAULT_PROFILES_FILE:-unset}"'
)


def test_configure_on_wsl_exports_profile(shell: ShellRunner) -> None:
    """On WSL mirrored networking both Fast DDS variables point to the repository profile."""
    result_ = Run_dds(shell, PRINT_PROFILES, ROUTE_WSL_MIRRORED)
    assert result_.returncode == 0, result_.stderr
    assert f"FASTRTPS={PROFILE_FILE}" in result_.stdout
    assert f"FASTDDS={PROFILE_FILE}" in result_.stdout
    assert "localhost is redirected to Windows" in result_.stdout


def test_configure_on_wsl_lets_the_profile_decide_discovery(shell: ShellRunner) -> None:
    """LOCALHOST would override the profile with redirected 127.0.0.1 peers: replaced."""
    snippet_ = (
        "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; vicon_configure_dds; "
        'echo "RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE}"'
    )
    result_ = Run_dds(shell, snippet_, ROUTE_WSL_MIRRORED)
    assert "RANGE=SYSTEM_DEFAULT" in result_.stdout
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST replaced by SYSTEM_DEFAULT" in result_.stdout


def test_configure_on_normal_machine_keeps_discovery_range(shell: ShellRunner) -> None:
    """Without the redirection the user's discovery range is untouched."""
    snippet_ = (
        "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; vicon_configure_dds; "
        'echo "RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE}"'
    )
    result_ = Run_dds(shell, snippet_, ROUTE_LOCAL)
    assert "RANGE=LOCALHOST" in result_.stdout


def test_configure_on_normal_machine_changes_nothing(shell: ShellRunner) -> None:
    """Elsewhere the default DDS configuration is kept."""
    result_ = Run_dds(shell, PRINT_PROFILES, ROUTE_LOCAL)
    assert result_.returncode == 0, result_.stderr
    assert "FASTRTPS=unset" in result_.stdout
    assert "FASTDDS=unset" in result_.stdout


def test_configure_keeps_profile_set_by_user(shell: ShellRunner) -> None:
    """A profile chosen by the user is never replaced."""
    result_ = Run_dds(
        shell, PRINT_PROFILES, ROUTE_WSL_MIRRORED, preset_profile="/home/user/my_profile.xml"
    )
    assert "FASTRTPS=/home/user/my_profile.xml" in result_.stdout
    assert "Keeping the Fast DDS profile already set" in result_.stdout


def test_configure_warns_without_wsl_loopback_address(shell: ShellRunner) -> None:
    """If lo lacks 10.255.255.254 the profile cannot work: warn and change nothing."""
    result_ = Run_dds(
        shell, PRINT_PROFILES, ROUTE_WSL_MIRRORED, lo_addresses=LO_WITHOUT_WSL_ADDRESS
    )
    assert result_.returncode == 0
    assert "FASTRTPS=unset" in result_.stdout
    assert "WARNING" in result_.stderr
    assert "10.255.255.254" in result_.stderr


@pytest.mark.parametrize("address", ["10.255.255.254"])
def test_profile_uses_the_wsl_loopback_address(address: str) -> None:
    """config/fastdds_wsl.xml and dds.sh agree on the loopback address."""
    profile_text_ = open(PROFILE_FILE, encoding="utf-8").read()
    dds_text_ = DDS_SH.read_text(encoding="utf-8")
    assert profile_text_.count(f"<address>{address}</address>") == 2  # whitelist + initial peer
    assert f'VICON_WSL_LOOPBACK_ADDRESS="{address}"' in dds_text_


# -------------------------------------------------------------------------------------------------
# source_ros_env.sh
# -------------------------------------------------------------------------------------------------
def test_ros_env_refuses_to_be_executed(shell: ShellRunner) -> None:
    """source_ros_env.sh only makes sense when sourced into the user's shell."""
    result_ = shell.run_script("source_ros_env.sh", [])
    assert result_.returncode == 1
    assert "must be sourced" in result_.stderr
