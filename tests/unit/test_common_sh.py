"""Unit tests of the shell helpers in scripts/lib/common.sh.

The network-dependent helpers are tested with the fake 'ip' and 'timeout' commands in
tests/fixtures/fake_bin, so every branch of the Vicon reachability check is deterministic and runs
without a Vicon system. One test uses a real local TCP listener for the success path.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import pytest

from conftest import ShellRunner, TcpListener

# Canned output of "ip -o -4 addr show" for a machine on the Vicon network (eth9) and an
# office network (eth4)
ADDR_ON_VICON_NETWORK: str = (
    "1: lo    inet 127.0.0.1/8 scope host lo\n"
    "4: eth4    inet 192.168.114.39/24 brd 192.168.114.255 scope global eth4\n"
    "9: eth9    inet 192.168.10.50/24 brd 192.168.10.255 scope global eth9"
)
# Same machine without the Vicon network card configured
ADDR_OFF_VICON_NETWORK: str = (
    "1: lo    inet 127.0.0.1/8 scope host lo\n"
    "4: eth4    inet 192.168.114.39/24 brd 192.168.114.255 scope global eth4"
)
ROUTE_DIRECT: str = "192.168.10.1 dev eth9 src 192.168.10.50 uid 1000"
ROUTE_VIA_GATEWAY: str = "192.168.10.1 via 192.168.114.1 dev eth4 src 192.168.114.39 uid 1000"


def Fake_network_env(
    addr_output: str = ADDR_OFF_VICON_NETWORK,
    route_output: str = ROUTE_VIA_GATEWAY,
    tcp_stderr: str = "",
    tcp_exit: int = 0,
    route_fails: bool = False,
) -> dict[str, str]:
    """Environment variables configuring the fake 'ip' and 'timeout' commands."""
    return {
        "FAKE_IP_ADDR_OUTPUT": addr_output,
        "FAKE_IP_ROUTE_OUTPUT": route_output,
        "FAKE_IP_ROUTE_FAIL": "1" if route_fails else "0",
        "FAKE_TIMEOUT_STDERR": tcp_stderr,
        "FAKE_TIMEOUT_EXIT": str(tcp_exit),
    }


# -------------------------------------------------------------------------------------------------
# is_ipv4_address
# -------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("address", ["192.168.10.1", "127.0.0.1", "10.0.0.255"])
def test_is_ipv4_address_accepts_dotted_quads(shell: ShellRunner, address: str) -> None:
    """Dotted IPv4 addresses are recognised as addresses (no name resolution needed)."""
    result_ = shell.run_common(f'is_ipv4_address "{address}"')
    assert result_.returncode == 0


@pytest.mark.parametrize("address", ["vicon-pc", "192.168.10", "192.168.10.1:801", ""])
def test_is_ipv4_address_rejects_other_strings(shell: ShellRunner, address: str) -> None:
    """Host names, truncated addresses and host:port strings are not plain addresses."""
    result_ = shell.run_common(f'is_ipv4_address "{address}" || echo "not an address"')
    assert "not an address" in result_.stdout


# -------------------------------------------------------------------------------------------------
# split_host_port
# -------------------------------------------------------------------------------------------------
@pytest.mark.parametrize(
    ("host_spec", "expected_host", "expected_port"),
    [
        ("192.168.10.1", "192.168.10.1", "801"),
        ("192.168.10.1:802", "192.168.10.1", "802"),
        ("vicon-pc", "vicon-pc", "801"),
    ],
)
def test_split_host_port(
    shell: ShellRunner, host_spec: str, expected_host: str, expected_port: str
) -> None:
    """The port defaults to the DataStream port 801 when the hostname has no ':port'."""
    result_ = shell.run_common(f'split_host_port "{host_spec}"; echo "$VICON_HOST $VICON_PORT"')
    assert result_.returncode == 0, result_.stderr
    assert result_.stdout.strip() == f"{expected_host} {expected_port}"


@pytest.mark.parametrize(
    ("host_spec", "expected_error"),
    [
        ("192.168.10.1;192.168.10.2", "several servers"),
        (":801", "Empty hostname"),
        ("192.168.10.1:abc", "Invalid port"),
    ],
)
def test_split_host_port_rejects_invalid_specs(
    shell: ShellRunner, host_spec: str, expected_error: str
) -> None:
    """Invalid hostname values stop the script with a message naming the problem."""
    result_ = shell.run_common(f'split_host_port "{host_spec}"')
    assert result_.returncode == 1
    assert expected_error in result_.stderr


# -------------------------------------------------------------------------------------------------
# read_config_param
# -------------------------------------------------------------------------------------------------
def test_read_config_param_reads_hostname(
    shell: ShellRunner, make_config: Callable[..., Path]
) -> None:
    """The hostname is read from vicon_client.ros__parameters."""
    config_ = make_config(overrides={"hostname": "10.1.2.3:900"})
    result_ = shell.run_common(f'read_config_param "{config_}" hostname')
    assert result_.returncode == 0, result_.stderr
    assert result_.stdout.strip() == "10.1.2.3:900"


def test_read_config_param_reads_repository_config(shell: ShellRunner) -> None:
    """The repository config points at the default Vicon Tracker address."""
    result_ = shell.run_common('read_config_param "$DEFAULT_CONFIG_FILE" hostname')
    assert result_.returncode == 0, result_.stderr
    assert result_.stdout.strip() == "192.168.10.1"


def test_read_config_param_missing_key(
    shell: ShellRunner, make_config: Callable[..., Path]
) -> None:
    """A missing parameter is reported with its full path in the file."""
    config_ = make_config(remove=["hostname"])
    result_ = shell.run_common(f'value=""; value="$(read_config_param "{config_}" hostname)"')
    assert result_.returncode == 1
    assert "vicon_client.ros__parameters.hostname not found" in result_.stderr


def test_read_config_param_malformed_yaml(shell: ShellRunner, tmp_path: Path) -> None:
    """A file that is not valid YAML is reported as unparsable."""
    config_ = tmp_path / "broken.yaml"
    config_.write_text("vicon_client: [unclosed\n", encoding="utf-8")
    result_ = shell.run_common(f'value=""; value="$(read_config_param "{config_}" hostname)"')
    assert result_.returncode == 1
    assert "cannot parse" in result_.stderr


def test_read_config_param_wrong_node_name(shell: ShellRunner, tmp_path: Path) -> None:
    """Parameters under another node name are not picked up."""
    config_ = tmp_path / "other_node.yaml"
    config_.write_text(
        "other_node:\n  ros__parameters:\n    hostname: '1.2.3.4'\n", encoding="utf-8"
    )
    result_ = shell.run_common(f'value=""; value="$(read_config_param "{config_}" hostname)"')
    assert result_.returncode == 1
    assert "not found" in result_.stderr


# -------------------------------------------------------------------------------------------------
# resolve_ipv4
# -------------------------------------------------------------------------------------------------
def test_resolve_ipv4_returns_literal_addresses(shell: ShellRunner) -> None:
    """IPv4 literals are returned unchanged, without a lookup."""
    result_ = shell.run_common('resolve_ipv4 "192.168.10.1"')
    assert result_.stdout.strip() == "192.168.10.1"


def test_resolve_ipv4_resolves_localhost(shell: ShellRunner) -> None:
    """Host names are resolved to an IPv4 address."""
    result_ = shell.run_common('resolve_ipv4 "localhost"')
    assert result_.stdout.strip().startswith("127.")


def test_resolve_ipv4_unknown_name_prints_nothing(shell: ShellRunner) -> None:
    """Unknown names produce no output and do not abort the caller."""
    result_ = shell.run_common('resolve_ipv4 "no-such-host.invalid"; echo "done"')
    assert result_.returncode == 0
    assert result_.stdout.strip() == "done"


# -------------------------------------------------------------------------------------------------
# describe_route / describe_local_subnet
# -------------------------------------------------------------------------------------------------
def test_describe_route_direct(shell: ShellRunner) -> None:
    """A directly connected route is printed as is."""
    result_ = shell.run_common(
        'describe_route "192.168.10.1"',
        extra_env=Fake_network_env(route_output=ROUTE_DIRECT),
        use_fake_bin=True,
    )
    assert result_.stdout.strip() == ROUTE_DIRECT


def test_describe_route_via_gateway_is_flagged(shell: ShellRunner) -> None:
    """A route through a gateway is flagged as not directly connected."""
    result_ = shell.run_common(
        'describe_route "192.168.10.1"',
        extra_env=Fake_network_env(route_output=ROUTE_VIA_GATEWAY),
        use_fake_bin=True,
    )
    assert "not a directly connected network" in result_.stdout


def test_describe_route_without_route(shell: ShellRunner) -> None:
    """A failing route lookup is reported as 'no route'."""
    result_ = shell.run_common(
        'describe_route "192.168.10.1"',
        extra_env=Fake_network_env(route_fails=True),
        use_fake_bin=True,
    )
    assert result_.stdout.strip() == "no route"


def test_describe_local_subnet_match(shell: ShellRunner) -> None:
    """The interface in the subnet of the Vicon PC is named."""
    result_ = shell.run_common(
        'describe_local_subnet "192.168.10.1"',
        extra_env=Fake_network_env(addr_output=ADDR_ON_VICON_NETWORK),
        use_fake_bin=True,
    )
    assert result_.stdout.strip() == "same subnet as 192.168.10.1 on: eth9 192.168.10.50/24"


def test_describe_local_subnet_no_match_lists_addresses(shell: ShellRunner) -> None:
    """Without a matching interface, the local addresses are listed (loopback excluded)."""
    result_ = shell.run_common(
        'describe_local_subnet "192.168.10.1"',
        extra_env=Fake_network_env(addr_output=ADDR_OFF_VICON_NETWORK),
        use_fake_bin=True,
    )
    output_ = result_.stdout.strip()
    assert output_.startswith("NO local interface is in the subnet of 192.168.10.1")
    assert "eth4 192.168.114.39/24" in output_
    assert "127.0.0.1" not in output_


# -------------------------------------------------------------------------------------------------
# check_vicon_reachable
# -------------------------------------------------------------------------------------------------
def test_check_vicon_reachable_succeeds_with_listener(
    shell: ShellRunner, tcp_listener: TcpListener
) -> None:
    """A server accepting TCP connections passes the check (real network stack)."""
    result_ = shell.run_common(
        f'check_vicon_reachable "{tcp_listener.host}" "{tcp_listener.port}"'
    )
    assert result_.returncode == 0, result_.stderr
    assert f"reachable at {tcp_listener.host}:{tcp_listener.port}" in result_.stdout


def test_check_vicon_reachable_unresolvable_name(shell: ShellRunner) -> None:
    """An unknown host name is reported before any connection attempt."""
    result_ = shell.run_common('check_vicon_reachable "no-such-host.invalid" 801')
    assert result_.returncode == 1
    assert "Cannot resolve Vicon host name 'no-such-host.invalid'" in result_.stderr


def test_check_vicon_reachable_no_route(shell: ShellRunner) -> None:
    """Without any route, the check stops before trying to connect."""
    result_ = shell.run_common(
        'check_vicon_reachable "192.168.10.1" 801',
        extra_env=Fake_network_env(route_fails=True),
        use_fake_bin=True,
    )
    assert result_.returncode == 1
    assert "No network route to the Vicon Tracker PC at 192.168.10.1" in result_.stderr
    assert "Checking Vicon DataStream server" not in result_.stdout


def test_check_vicon_reachable_connection_refused(shell: ShellRunner) -> None:
    """A refused connection means the PC is up but DataStream is not listening."""
    result_ = shell.run_common(
        'check_vicon_reachable "192.168.10.1" 801',
        extra_env=Fake_network_env(
            route_output=ROUTE_DIRECT,
            tcp_stderr="bash: connect: Connection refused",
            tcp_exit=1,
        ),
        use_fake_bin=True,
    )
    assert result_.returncode == 1
    assert "is up, but nothing accepts connections on TCP 801" in result_.stderr
    assert "DataStream server enabled" in result_.stderr


@pytest.mark.parametrize(
    ("tcp_stderr", "tcp_exit", "expected_cause"),
    [
        ("", 124, "no answer within 3 s"),
        ("bash: connect: No route to host", 1, "no route to host"),
        ("bash: connect: Network is unreachable", 1, "network unreachable"),
    ],
)
def test_check_vicon_reachable_unreachable_causes(
    shell: ShellRunner, tcp_stderr: str, tcp_exit: int, expected_cause: str
) -> None:
    """Each connection failure is translated into its cause, with subnet and route hints."""
    result_ = shell.run_common(
        'check_vicon_reachable "192.168.10.1" 801',
        extra_env=Fake_network_env(tcp_stderr=tcp_stderr, tcp_exit=tcp_exit),
        use_fake_bin=True,
    )
    assert result_.returncode == 1
    assert f"Vicon Tracker PC not reachable at 192.168.10.1:801: {expected_cause}" in (
        result_.stderr
    )
    # The diagnostic hints are part of the message
    assert "Local network: NO local interface is in the subnet of 192.168.10.1" in result_.stderr
    assert "Route used:" in result_.stderr


def test_check_vicon_reachable_uses_configured_timeout(shell: ShellRunner) -> None:
    """VICON_CHECK_TIMEOUT_S sets the timeout reported in the messages."""
    environment_ = Fake_network_env(tcp_exit=124)
    environment_["VICON_CHECK_TIMEOUT_S"] = "7"
    result_ = shell.run_common(
        'check_vicon_reachable "192.168.10.1" 801', extra_env=environment_, use_fake_bin=True
    )
    assert "(timeout 7 s)" in result_.stdout
    assert "no answer within 7 s" in result_.stderr
