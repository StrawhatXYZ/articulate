"""Small helpers for spinning a node against simulation time."""

import time
from typing import Callable

import rclpy
from rclpy.node import Node

from .tasks import SimulationUnresponsive


def sim_now(node: Node) -> float:
    """Return the node's current time in seconds (simulation time with use_sim_time)."""
    return node.get_clock().now().nanoseconds / 1e9


def spin_until(node: Node, predicate: Callable[[], bool], sim_timeout_s: float,
               wall_timeout_s: float = 60.0) -> bool:
    """
    Spin node until predicate() is true.

    Returns False if sim_timeout_s of simulation time passes first. Raises
    SimulationUnresponsive if wall_timeout_s of wall time passes first, which
    means the simulation has stalled.
    """
    sim_start = sim_now(node)
    wall_start = time.monotonic()
    while not predicate():
        if sim_now(node) - sim_start >= sim_timeout_s:
            return False
        if time.monotonic() - wall_start >= wall_timeout_s:
            raise SimulationUnresponsive(f'no progress for {wall_timeout_s:.0f} s of wall time')
        rclpy.spin_once(node, timeout_sec=0.01)
    return True


def spin_for(node: Node, sim_seconds: float) -> None:
    """Spin node for sim_seconds of simulation time."""
    spin_until(node, lambda: False, sim_seconds)
