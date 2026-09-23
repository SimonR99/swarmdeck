"""Fleet ROS topics are received once per process and distributed in callback order."""

from types import SimpleNamespace as NS
from unittest.mock import Mock
import sys


def test_peer_coordinators_share_fleet_subscriptions_and_authority_reader(monkeypatch):
    from adapters.peer_coordination import PeerCoordinator

    monkeypatch.setitem(sys.modules, "std_msgs.msg", NS(String=lambda **kw: NS(**kw)))
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", NS(Pose=NS, PoseArray=NS))
    subscriptions = {}

    def subscribe(_type, topic, callback, _depth):
        subscriptions[topic] = callback
        return object()

    node = NS(create_subscription=Mock(side_effect=subscribe),
              create_publisher=lambda *args: NS(publish=lambda message: None))
    bridges = [NS(node=node, id=f"r{i}", navigation_frame=f"r{i}/odom") for i in range(4)]
    coordinators = [PeerCoordinator(bridge, {}) for bridge in bridges]
    assert len(subscriptions) == 2 + 4  # Two fleet topics and four distinct authority topics.
    for topic in ("/swarmdeck/intentions", "/swarmdeck/exploration_reports"):
        assert sum(call.args[1] == topic for call in node.create_subscription.call_args_list) == 1
    assert all(sum(call.args[1] == f"/r{i}/map_authority"
                   for call in node.create_subscription.call_args_list) == 1 for i in range(4))
    assert all(c.authority_subscription is c.mapping_authority.subscription for c in coordinators)



def test_shared_subscription_fans_out_once_per_message_in_registration_order():
    from adapters.mapping_authority import shared_subscription

    node = NS(create_subscription=Mock(return_value=object()))
    seen = []
    first = shared_subscription(node, str, "/swarmdeck/intentions", lambda m: seen.append((1, m)), 20)
    second = shared_subscription(node, str, "/swarmdeck/intentions", lambda m: seen.append((2, m)), 20)
    assert first is second
    node.create_subscription.assert_called_once()
    node.create_subscription.call_args.args[2]("message")
    assert seen == [(1, "message"), (2, "message")]


def test_authority_subscriptions_keep_distinct_robot_topics():
    from adapters.mapping_authority import shared_subscription

    node = NS(create_subscription=Mock(return_value=object()))
    shared_subscription(node, str, "/r0/map_authority", lambda m: None, 5)
    shared_subscription(node, str, "/r1/map_authority", lambda m: None, 5)
    assert node.create_subscription.call_count == 2
