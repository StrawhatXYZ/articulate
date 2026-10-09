"""Spinning against simulation time after the node has been idle."""

from lab_simulation import ros_util


class IdleNode:
    """A node whose clock is stale until spun: the first spin jumps 60 s ahead."""

    def __init__(self):
        self.t, self.spins = 100.0, 0

    def get_clock(self):
        node = self

        class Clock:
            def now(self):
                class Time:
                    nanoseconds = int(node.t * 1e9)
                return Time()
        return Clock()

    def spin_once(self):
        self.spins += 1
        self.t += 60.0 if self.spins == 1 else 0.01


def test_spin_until_measures_from_the_current_time_not_a_stale_one(monkeypatch):
    node = IdleNode()
    monkeypatch.setattr(ros_util.rclpy, 'spin_once', lambda n, timeout_sec=0.0: n.spin_once())
    assert ros_util.spin_until(node, lambda: node.spins >= 5, sim_timeout_s=20.0)


def test_catch_up_brings_the_clock_up_to_date(monkeypatch):
    node = IdleNode()
    monkeypatch.setattr(ros_util.rclpy, 'spin_once', lambda n, timeout_sec=0.0: n.spin_once())
    ros_util.catch_up(node)
    assert node.t >= 160.0


def test_a_node_spun_just_now_is_not_caught_up_again(monkeypatch):
    node = IdleNode()
    monkeypatch.setattr(ros_util.rclpy, 'spin_once', lambda n, timeout_sec=0.0: n.spin_once())
    ros_util.spin_until(node, lambda: node.spins >= 60, sim_timeout_s=1000.0)
    before = node.spins
    ros_util.spin_until(node, lambda: True, sim_timeout_s=1.0)
    assert node.spins == before
