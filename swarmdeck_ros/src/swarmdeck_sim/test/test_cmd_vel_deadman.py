"""A latched velocity has to expire.

The ARGoS controller holds whatever velocity it was last given, and the bridge
resends its newest `cmd_vel` on every exchange. Without a timeout a robot whose
publisher stops keeps driving on the last command forever, with its mode
reading `idle` because nothing in the adapter ever saw a drive.

That was measured, not imagined: one `cmd_vel` of 0.25 m/s and no further
publication left the robot doing 0.25 m/s forty seconds later.

The module needs rclpy, which the ROS-free venv does not have, so the logic
under test is exercised through a stand-in with the same shape. What is being
checked is the expiry rule itself, which is where the bug was.
"""

from __future__ import annotations

import pytest

# Simulation seconds, mirroring CMD_VEL_TIMEOUT_SIM_S in the bridge. Restated
# rather than imported because importing the bridge needs a ROS distribution.
TIMEOUT = 0.5


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


class _Node:
    def __init__(self):
        self._logger = _Logger()

    def get_logger(self):
        return self._logger


class Interface:
    """The bridge's per-robot command state, with no ROS in it.

    Kept byte-for-byte equivalent to RobotInterface.velocity_at(); if that
    changes, this must change with it.
    """

    def __init__(self):
        self.node = _Node()
        self.id = "robot_0"
        self.cmd_vel = (0.0, 0.0)
        self.cmd_seq = 0
        self._seen_seq = 0
        self._cmd_at_sim = None
        self._expired = False

    def receive(self, linear, angular):
        self.cmd_vel = (linear, angular)
        self.cmd_seq += 1

    def velocity_at(self, sim_now):
        if self.cmd_seq != self._seen_seq:
            self._seen_seq = self.cmd_seq
            self._cmd_at_sim = sim_now
            if self._expired:
                self._expired = False
        if self._cmd_at_sim is None:
            return (0.0, 0.0)
        if sim_now - self._cmd_at_sim <= TIMEOUT:
            return self.cmd_vel
        if not self._expired and self.cmd_vel != (0.0, 0.0):
            self._expired = True
            self.node.get_logger().info(f"[{self.id}] no cmd_vel; commanding zero")
        return (0.0, 0.0)


def test_a_command_is_held_while_it_is_fresh():
    """Publishers do not command every tick, and the gaps are normal."""
    iface = Interface()
    iface.receive(0.25, 0.0)
    assert iface.velocity_at(10.0) == (0.25, 0.0)
    assert iface.velocity_at(10.0 + TIMEOUT / 2) == (0.25, 0.0)
    assert iface.velocity_at(10.0 + TIMEOUT) == (0.25, 0.0)


def test_a_stale_command_expires_to_zero():
    """The regression. One command, no publisher, and the robot drove forever."""
    iface = Interface()
    iface.receive(0.25, 0.0)
    assert iface.velocity_at(10.0) == (0.25, 0.0)
    assert iface.velocity_at(10.0 + TIMEOUT + 0.01) == (0.0, 0.0)
    # And stays zero, however long nothing publishes.
    assert iface.velocity_at(1000.0) == (0.0, 0.0)


def test_expiry_is_reported_once_rather_than_every_tick():
    """A robot parked with no publisher would otherwise fill the log forever."""
    iface = Interface()
    iface.receive(0.25, 0.0)
    for step in range(200):
        iface.velocity_at(10.0 + TIMEOUT + 0.01 + step * 0.1)
    assert len(iface.node.get_logger().messages) == 1


def test_the_clock_starts_when_a_command_is_first_OBSERVED():
    """Arrival time is taken at the first send after receipt, not in the ROS
    callback, which runs on a thread with no access to the simulation clock. A
    command is therefore always fresh the first time it is acted on, however
    long it sat in the queue."""
    iface = Interface()
    iface.receive(0.25, 0.0)
    assert iface.velocity_at(9999.0) == (0.25, 0.0)
    assert iface.velocity_at(9999.0 + TIMEOUT + 0.01) == (0.0, 0.0)


def test_a_fresh_command_revives_an_expired_robot():
    """Expiry must not latch: exploration resuming after a goal ends, or an
    operator driving after a pause, has to work without a restart."""
    iface = Interface()
    iface.receive(0.30, 0.1)
    assert iface.velocity_at(20.0) == (0.30, 0.1)
    assert iface.velocity_at(20.0 + TIMEOUT + 0.01) == (0.0, 0.0)

    iface.receive(0.10, 0.0)
    assert iface.velocity_at(30.0) == (0.10, 0.0)
    assert iface.velocity_at(30.0 + TIMEOUT + 0.01) == (0.0, 0.0)
    # Two lapses, two lines.
    assert len(iface.node.get_logger().messages) == 2


def test_nothing_is_commanded_before_the_first_message():
    """A robot must not inherit a velocity it was never given."""
    iface = Interface()
    assert iface.velocity_at(0.0) == (0.0, 0.0)
    assert iface.velocity_at(9999.0) == (0.0, 0.0)


def test_a_zero_command_expiring_is_not_worth_a_log_line():
    """Stopping a stopped robot changes nothing and says nothing."""
    iface = Interface()
    iface.receive(0.0, 0.0)
    assert iface.velocity_at(10.0 + TIMEOUT + 0.01) == (0.0, 0.0)
    assert iface.node.get_logger().messages == []


@pytest.mark.parametrize("rate_hz", [20.0, 10.0, 2.0])
def test_a_publisher_at_a_normal_rate_is_never_chopped(rate_hz):
    """Nav2 runs at ~20 Hz of SIMULATION time whatever the real-time factor is,
    which is the binding constraint on this timeout; explore.py publishes on the
    wall clock and so appears FASTER in sim time as the simulation slows."""
    iface = Interface()
    period = 1.0 / rate_hz
    now = 5.0
    for _ in range(50):
        iface.receive(0.4, 0.0)
        now += period
        assert iface.velocity_at(now) == (0.4, 0.0), f"chopped at {rate_hz} Hz"
