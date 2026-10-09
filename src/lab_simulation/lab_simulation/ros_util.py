"""Small helpers for spinning a node against simulation time."""

import math
import time
from typing import Callable

import rclpy
from rclpy.node import Node

from .tasks import SimulationUnresponsive

# Callbacks processed to bring an idle node up to date (zero-timeout spins:
# each handles at most one queued message, and costs nothing when none is).
CATCH_UP_SPINS = 50
IDLE_S = 0.5             # wall seconds without spinning after which a node is stale
_last_spin = {}          # id(node) -> wall time it was last spun here


def sim_now(node: Node) -> float:
    """Return the node's current time in seconds (simulation time with use_sim_time)."""
    return node.get_clock().now().nanoseconds / 1e9


def catch_up(node: Node) -> None:
    """
    Process the messages queued while node was not spun.

    A node that nobody spun for a while (between two instructions, say) has
    a stale simulation clock: measuring a timeout or "a frame newer than
    now" from it would be wrong. Catch up before reading the time.
    """
    if time.monotonic() - _last_spin.get(id(node), -math.inf) < IDLE_S:
        return
    for _ in range(CATCH_UP_SPINS):
        rclpy.spin_once(node, timeout_sec=0.0)
    _last_spin[id(node)] = time.monotonic()


def spin_until(node: Node, predicate: Callable[[], bool], sim_timeout_s: float,
               wall_timeout_s: float = 60.0) -> bool:
    """
    Spin node until predicate() is true.

    Returns False if sim_timeout_s of simulation time passes first. Raises
    SimulationUnresponsive if wall_timeout_s of wall time passes first, which
    means the simulation has stalled.
    """
    catch_up(node)
    sim_start = sim_now(node)
    wall_start = time.monotonic()
    while not predicate():
        if sim_now(node) - sim_start >= sim_timeout_s:
            return False
        if time.monotonic() - wall_start >= wall_timeout_s:
            raise SimulationUnresponsive(f'no progress for {wall_timeout_s:.0f} s of wall time')
        rclpy.spin_once(node, timeout_sec=0.01)
        _last_spin[id(node)] = time.monotonic()
    return True


def spin_for(node: Node, sim_seconds: float) -> None:
    """Spin node for sim_seconds of simulation time."""
    spin_until(node, lambda: False, sim_seconds)
