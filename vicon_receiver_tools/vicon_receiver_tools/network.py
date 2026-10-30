"""Diagnostics of the local network for the Vicon reachability check.

A Vicon system usually sits on its own network: the machine running the node then needs an
address in the same IPv4 subnet as the Vicon Tracker PC. This module tells whether that is the
case, from the output of ``ip -o -4 addr show``.

Command line (used by scripts/lib/common.sh)::

    ip -o -4 addr show | python3 -m vicon_receiver_tools.network TARGET_ADDRESS

prints one line and exits with 0, or exits with 1 if TARGET_ADDRESS is not an IPv4 address.

Example::

    $ ip -o -4 addr show | python3 -m vicon_receiver_tools.network 192.168.10.1
    NO local interface is in the subnet of 192.168.10.1 (local addresses: eth4 192.168.114.39/24)
"""

from __future__ import annotations

import argparse
import ipaddress
import sys
from collections.abc import Iterable
from dataclasses import dataclass

# Name of the loopback device, never relevant for reaching another machine
LOOPBACK_DEVICE: str = "lo"


@dataclass(frozen=True)
class InterfaceAddress:
    """One IPv4 address assigned to a local network device.

    Attributes:
        device: Device name, e.g. "eth0".
        interface: Address with its prefix, e.g. 192.168.10.50/24.
    """

    device: str
    interface: ipaddress.IPv4Interface

    def is_loopback(self) -> bool:
        """Whether the address belongs to the loopback device or range."""
        return self.device == LOOPBACK_DEVICE or self.interface.ip.is_loopback

    def __str__(self) -> str:
        return f"{self.device} {self.interface.with_prefixlen}"


def Parse_ip_addr_output(lines: Iterable[str]) -> list[InterfaceAddress]:
    """Parse the output of ``ip -o -4 addr show``.

    Each line looks like ``3: eth4    inet 192.168.114.39/24 brd 192.168.114.255 scope global``.
    Lines that do not have this shape are skipped.

    Args:
        lines: Output lines of the command.

    Returns:
        The IPv4 addresses, in the order of the input.
    """
    addresses_: list[InterfaceAddress] = []
    for line_ in lines:
        fields_ = line_.split()
        if len(fields_) < 4 or fields_[2] != "inet":
            continue

        try:
            interface_ = ipaddress.IPv4Interface(fields_[3])
        except ValueError:
            continue

        addresses_.append(InterfaceAddress(device=fields_[1], interface=interface_))
    return addresses_


def Describe_local_subnet(
    target: ipaddress.IPv4Address, addresses: Iterable[InterfaceAddress]
) -> str:
    """Describe whether a local interface is in the same subnet as the target address.

    Args:
        target: Address of the remote machine (the Vicon Tracker PC).
        addresses: Local addresses, e.g. from Parse_ip_addr_output().

    Returns:
        One line naming the interfaces in the subnet of the target, or listing all the local
        addresses (loopback excluded) if there is none.
    """
    same_subnet_: list[InterfaceAddress] = []
    other_addresses_: list[InterfaceAddress] = []
    for address_ in addresses:
        if address_.is_loopback():
            continue
        if target in address_.interface.network:
            same_subnet_.append(address_)
        else:
            other_addresses_.append(address_)

    if same_subnet_:
        names_ = ", ".join(str(address_) for address_ in same_subnet_)
        return f"same subnet as {target} on: {names_}"

    names_ = ", ".join(str(address_) for address_ in other_addresses_)
    return f"NO local interface is in the subnet of {target} (local addresses: {names_})"


def Main(argv: list[str] | None = None) -> int:
    """Command-line entry point: read ``ip -o -4 addr show`` on stdin, print the report.

    Returns:
        Process exit code: 0 on success, 1 if the target is not an IPv4 address.
    """
    parser_ = argparse.ArgumentParser(
        prog="python3 -m vicon_receiver_tools.network",
        description="Tell whether a local interface is in the subnet of an IPv4 address "
        "(reads the output of 'ip -o -4 addr show' on stdin).",
    )
    parser_.add_argument("target", help="IPv4 address of the remote machine")
    arguments_ = parser_.parse_args(argv)

    try:
        target_ = ipaddress.IPv4Address(arguments_.target)
    except ValueError:
        print(f"not an IPv4 address: {arguments_.target}", file=sys.stderr)
        return 1

    addresses_ = Parse_ip_addr_output(sys.stdin)
    print(Describe_local_subnet(target_, addresses_))
    return 0


if __name__ == "__main__":
    sys.exit(Main())
