"""Unit tests of vicon_receiver_tools.network (local subnet diagnostics)."""

from __future__ import annotations

import io
import ipaddress

import pytest

from vicon_receiver_tools.network import (
    InterfaceAddress,
    Describe_local_subnet,
    Main,
    Parse_ip_addr_output,
)

VICON_PC: ipaddress.IPv4Address = ipaddress.IPv4Address("192.168.10.1")

# Output of "ip -o -4 addr show" on a WSL machine (with an address on the loopback device)
# connected to the Vicon network on eth9
IP_ADDR_OUTPUT: list[str] = [
    "1: lo    inet 127.0.0.1/8 scope host lo",
    "1: lo    inet 10.255.255.254/32 brd 10.255.255.254 scope global lo",
    "4: eth4    inet 192.168.114.39/24 brd 192.168.114.255 scope global noprefixroute eth4",
    "9: eth9    inet 192.168.10.50/24 brd 192.168.10.255 scope global eth9",
]


def Make_address(device: str, cidr: str) -> InterfaceAddress:
    """Build an InterfaceAddress from a device name and an address/prefix string."""
    return InterfaceAddress(device=device, interface=ipaddress.IPv4Interface(cidr))


# -------------------------------------------------------------------------------------------------
# Parse_ip_addr_output
# -------------------------------------------------------------------------------------------------
def test_parse_ip_addr_output() -> None:
    """Device names and addresses with prefixes are extracted in order."""
    addresses_ = Parse_ip_addr_output(IP_ADDR_OUTPUT)
    assert [str(address_) for address_ in addresses_] == [
        "lo 127.0.0.1/8",
        "lo 10.255.255.254/32",
        "eth4 192.168.114.39/24",
        "eth9 192.168.10.50/24",
    ]


@pytest.mark.parametrize(
    "line",
    [
        "",
        "garbage",
        "2: eth0    inet6 fe80::1/64 scope link",
        "3: eth1    inet not-an-address brd 0.0.0.0",
    ],
)
def test_parse_ip_addr_output_skips_other_lines(line: str) -> None:
    """Lines that are not IPv4 address entries are ignored."""
    assert Parse_ip_addr_output([line]) == []


def test_loopback_detection() -> None:
    """Both the loopback device and the loopback range count as loopback."""
    assert Make_address("lo", "10.255.255.254/32").is_loopback()
    assert Make_address("dummy0", "127.0.0.2/8").is_loopback()
    assert not Make_address("eth0", "192.168.10.50/24").is_loopback()


# -------------------------------------------------------------------------------------------------
# Describe_local_subnet
# -------------------------------------------------------------------------------------------------
def test_describe_local_subnet_with_matching_interface() -> None:
    """The interface in the subnet of the Vicon PC is named."""
    report_ = Describe_local_subnet(VICON_PC, Parse_ip_addr_output(IP_ADDR_OUTPUT))
    assert report_ == "same subnet as 192.168.10.1 on: eth9 192.168.10.50/24"


def test_describe_local_subnet_without_matching_interface() -> None:
    """Without a match, the non-loopback local addresses are listed."""
    addresses_ = Parse_ip_addr_output(IP_ADDR_OUTPUT[:3])
    report_ = Describe_local_subnet(VICON_PC, addresses_)
    assert report_ == (
        "NO local interface is in the subnet of 192.168.10.1 "
        "(local addresses: eth4 192.168.114.39/24)"
    )


def test_describe_local_subnet_respects_prefix_length() -> None:
    """A /16 network contains the Vicon PC, a /24 on another subnet does not."""
    wide_ = Make_address("eth2", "192.168.0.7/16")
    narrow_ = Make_address("eth3", "192.168.20.7/24")
    report_ = Describe_local_subnet(VICON_PC, [wide_, narrow_])
    assert report_ == "same subnet as 192.168.10.1 on: eth2 192.168.0.7/16"


# -------------------------------------------------------------------------------------------------
# Command line
# -------------------------------------------------------------------------------------------------
def test_main_reads_stdin(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """The command reads 'ip -o -4 addr show' from stdin and prints the report."""
    monkeypatch.setattr("sys.stdin", io.StringIO("\n".join(IP_ADDR_OUTPUT) + "\n"))
    exit_code_ = Main(["192.168.10.1"])
    assert exit_code_ == 0
    assert capsys.readouterr().out == "same subnet as 192.168.10.1 on: eth9 192.168.10.50/24\n"


def test_main_rejects_invalid_target(capsys: pytest.CaptureFixture[str]) -> None:
    """A target that is not an IPv4 address is an error."""
    exit_code_ = Main(["vicon-pc"])
    assert exit_code_ == 1
    assert "not an IPv4 address: vicon-pc" in capsys.readouterr().err
