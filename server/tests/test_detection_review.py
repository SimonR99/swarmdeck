from __future__ import annotations

import math

from swarmdeck_server.detect.review import MAX_PENDING, ReviewStore


def store() -> ReviewStore:
    return ReviewStore(same_radius=0.5, ask_radius=1.5, ignore_radius=1.0)


def test_a_first_sighting_asks_rather_than_placing_itself():
    s = store()
    outcome, proposal = s.observe("robot_0", "rubber_duck", 2.0, 3.0, 0.8)

    assert outcome == "proposed"
    assert proposal is not None
    assert not s.entities, "nothing reaches the map without an operator saying so"
    assert proposal.suggested_entity_id is None


def test_an_object_already_on_the_map_never_asks_again():
    """The common case by a wide margin, and the whole point of the radius.

    A robot parked in front of a duck emits a sighting every frame. Prompting
    per frame would make the queue useless, so a sighting inside `same_radius`
    of a confirmed entity is folded in silently.
    """
    s = store()
    _, proposal = s.observe("robot_0", "rubber_duck", 2.0, 3.0, 0.8)
    entity = s.accept(proposal.id)

    for _ in range(20):
        outcome, target = s.observe("robot_0", "rubber_duck", 2.1, 3.05, 0.7)
        assert outcome == "folded"
        assert target is entity

    assert not s.proposals
    assert entity.acc.count == 21


def test_an_ambiguous_sighting_arrives_with_the_merge_already_suggested():
    s = store()
    _, first = s.observe("robot_0", "rubber_duck", 0.0, 0.0, 0.9)
    entity = s.accept(first.id)

    # Past `same_radius`, inside `ask_radius`: probably the same duck seen from
    # a worse angle, possibly a second one. That is the operator's call.
    outcome, proposal = s.observe("robot_1", "rubber_duck", 1.0, 0.0, 0.6)

    assert outcome == "proposed"
    assert proposal.suggested_entity_id == entity.id
    assert math.isclose(proposal.suggested_distance, 1.0, abs_tol=1e-6)


def test_a_distant_sighting_is_proposed_as_a_new_object():
    s = store()
    _, first = s.observe("robot_0", "rubber_duck", 0.0, 0.0, 0.9)
    s.accept(first.id)

    _, proposal = s.observe("robot_1", "rubber_duck", 5.0, 0.0, 0.8)

    assert proposal.suggested_entity_id is None


def test_a_different_class_at_the_same_spot_is_never_folded():
    """Two objects can share a location; a duck is not a cone."""
    s = store()
    _, first = s.observe("robot_0", "rubber_duck", 1.0, 1.0, 0.9)
    s.accept(first.id)

    outcome, proposal = s.observe("robot_0", "disc_cone", 1.0, 1.0, 0.9)

    assert outcome == "proposed"
    assert proposal.suggested_entity_id is None


def test_position_is_the_mean_of_every_accepted_observation():
    """A single monocular range estimate is worth little; the average is not.

    Running sums mean the centroid covers all evidence, not a recent window.
    """
    s = store()
    _, proposal = s.observe("robot_0", "rubber_duck", 0.0, 0.0, 0.5)
    entity = s.accept(proposal.id)
    s.observe("robot_1", "rubber_duck", 0.4, 0.0, 0.5)
    s.observe("robot_1", "rubber_duck", 0.2, 0.3, 0.5)

    assert entity.acc.count == 3
    assert math.isclose(entity.acc.x, 0.2, abs_tol=1e-9)
    assert math.isclose(entity.acc.y, 0.1, abs_tol=1e-9)
    assert entity.acc.robots == {"robot_0", "robot_1"}


def test_repeat_sightings_strengthen_one_proposal_instead_of_queueing_copies():
    s = store()
    s.observe("robot_0", "rubber_duck", 4.0, 4.0, 0.4)
    for _ in range(10):
        outcome, _ = s.observe("robot_0", "rubber_duck", 4.05, 4.02, 0.9)
        assert outcome == "updated"

    assert len(s.proposals) == 1
    only = next(iter(s.proposals.values()))
    assert only.acc.count == 11
    assert only.acc.best_score == 0.9


def test_merging_moves_the_evidence_not_just_the_marker():
    s = store()
    _, first = s.observe("robot_0", "rubber_duck", 0.0, 0.0, 0.9)
    entity = s.accept(first.id)
    _, second = s.observe("robot_1", "rubber_duck", 1.0, 0.0, 0.7)

    merged = s.merge(second.id, entity.id)

    assert merged is entity
    assert entity.acc.count == 2
    assert math.isclose(entity.acc.x, 0.5, abs_tol=1e-9)
    assert entity.acc.robots == {"robot_0", "robot_1"}
    assert not s.proposals


def test_merge_refuses_to_cross_classes():
    s = store()
    _, first = s.observe("robot_0", "rubber_duck", 0.0, 0.0, 0.9)
    entity = s.accept(first.id)
    _, cone = s.observe("robot_0", "disc_cone", 0.2, 0.0, 0.9)

    assert s.merge(cone.id, entity.id) is None
    assert cone.id in s.proposals, "a refused merge must not eat the proposal"


def test_ignore_sticks_instead_of_asking_again_next_frame():
    """Without the zone, `ignore` degrades into `dismiss`.

    The detector re-proposes the same object on the following frame, and the
    operator has to keep answering forever.
    """
    s = store()
    _, proposal = s.observe("robot_0", "wooden_block", -3.0, 2.0, 0.6)

    assert s.ignore(proposal.id) is True
    assert not s.proposals

    for _ in range(10):
        outcome, target = s.observe("robot_0", "wooden_block", -3.02, 2.03, 0.6)
        assert outcome == "suppressed"
        assert target is None

    assert not s.proposals and not s.entities


def test_ignoring_one_spot_does_not_silence_the_rest_of_the_class():
    s = store()
    _, proposal = s.observe("robot_0", "wooden_block", 0.0, 0.0, 0.6)
    s.ignore(proposal.id)

    outcome, _ = s.observe("robot_0", "wooden_block", 6.0, 6.0, 0.6)

    assert outcome == "proposed"


def test_a_flood_of_junk_cannot_push_out_the_strongest_finding():
    s = store()
    _, keeper = s.observe("robot_0", "rubber_duck", 0.0, 0.0, 0.99)
    for i in range(MAX_PENDING + 10):
        s.observe("robot_0", "rubber_duck", 10.0 + 2.0 * i, 0.0, 0.10)

    assert len(s.proposals) <= MAX_PENDING
    assert keeper.id in s.proposals


def test_switching_a_class_off_forgets_what_it_had_already_placed():
    s = store()
    _, duck = s.observe("robot_0", "rubber_duck", 0.0, 0.0, 0.9)
    s.accept(duck.id)
    _, cone = s.observe("robot_0", "disc_cone", 5.0, 5.0, 0.9)
    s.ignore(cone.id)
    s.observe("robot_0", "pool_noodle", 8.0, 8.0, 0.9)

    s.drop_classes({"pool_noodle"})

    assert not s.entities
    assert not s.ignored
    assert [p.cls for p in s.proposals.values()] == ["pool_noodle"]


def test_forget_removes_a_mistaken_acceptance():
    s = store()
    _, proposal = s.observe("robot_0", "rubber_duck", 1.0, 1.0, 0.9)
    entity = s.accept(proposal.id)

    assert s.forget(entity.id) is True
    assert s.forget(entity.id) is False
    assert not s.entities


def test_snapshot_reports_what_the_dashboard_needs():
    s = store()
    _, proposal = s.observe("robot_0", "rubber_duck", 1.0, 2.0, 0.75)
    s.accept(proposal.id)
    s.observe("robot_1", "disc_cone", -4.0, 0.0, 0.5)

    snap = s.snapshot()

    assert len(snap["entities"]) == 1 and len(snap["proposals"]) == 1
    assert snap["entities"][0]["position"] == {"x": 1.0, "y": 2.0}
    assert snap["entities"][0]["robot_ids"] == ["robot_0"]
    assert snap["radii"]["same"] == 0.5
