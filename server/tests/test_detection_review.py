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


def test_a_parked_robot_cannot_own_an_objects_position():
    """Frames are not independent measurements.

    Measured on a live overnight run before this gate existed: one idling robot
    contributed 176,563 sightings of one duck and dragged the marker from 0.17 m
    of error out to 0.39 m. Averaging is supposed to buy accuracy.
    """
    s = store()
    _, proposal = s.observe("robot_0", "rubber_duck", 0.0, 0.0, 0.9, observer=(2.0, 0.0))
    entity = s.accept(proposal.id)

    # Same pose, 500 more frames, each with the same biased estimate.
    for _ in range(500):
        s.observe("robot_0", "rubber_duck", 0.40, 0.0, 0.9, observer=(2.0, 0.0))

    assert entity.acc.sightings == 501, "every frame is still counted as a sighting"
    assert entity.acc.count == 1, "but a stationary robot is one viewpoint"
    assert math.isclose(entity.acc.x, 0.0, abs_tol=1e-9), "the centroid did not move"


def test_moving_to_a_new_vantage_point_does_count():
    s = store()
    _, proposal = s.observe("robot_0", "rubber_duck", 0.0, 0.0, 0.9, observer=(2.0, 0.0))
    entity = s.accept(proposal.id)

    # Drove around to the far side and saw it again.
    s.observe("robot_0", "rubber_duck", 0.40, 0.0, 0.9, observer=(-2.0, 0.0))

    assert entity.acc.count == 2
    assert math.isclose(entity.acc.x, 0.2, abs_tol=1e-9)


def test_a_second_robot_always_contributes_its_first_look():
    """A different robot is a different viewpoint by definition."""
    s = store()
    _, proposal = s.observe("robot_0", "rubber_duck", 0.0, 0.0, 0.9, observer=(2.0, 0.0))
    entity = s.accept(proposal.id)

    # robot_1 happens to be standing exactly where robot_0 is.
    s.observe("robot_1", "rubber_duck", 0.40, 0.0, 0.9, observer=(2.0, 0.0))

    assert entity.acc.count == 2
    assert entity.acc.robots == {"robot_0", "robot_1"}


def test_the_gate_is_per_object_not_global():
    """Two ducks seen from one pose must both get that pose's evidence."""
    s = store()
    _, a = s.observe("robot_0", "rubber_duck", 0.0, 0.0, 0.9, observer=(1.0, 0.0))
    _, b = s.observe("robot_0", "rubber_duck", 6.0, 0.0, 0.9, observer=(1.0, 0.0))

    assert a.acc.count == 1 and b.acc.count == 1
    assert a.id != b.id


def test_omitting_the_observer_keeps_averaging_every_frame():
    """Callers without a pose must not silently lose all their evidence."""
    s = store()
    _, proposal = s.observe("robot_0", "rubber_duck", 0.0, 0.0, 0.9)
    entity = s.accept(proposal.id)
    for _ in range(4):
        s.observe("robot_0", "rubber_duck", 0.5, 0.0, 0.9)

    assert entity.acc.count == 5
    assert math.isclose(entity.acc.x, 0.4, abs_tol=1e-9)


def test_state_survives_a_round_trip_through_disk():
    s = store()
    _, first = s.observe("robot_0", "rubber_duck", 1.0, 2.0, 0.9, observer=(0.0, 0.0))
    entity = s.accept(first.id)
    s.observe("robot_1", "rubber_duck", 1.4, 2.0, 0.7, observer=(5.0, 0.0))
    _, cone = s.observe("robot_0", "disc_cone", -6.0, 0.0, 0.5, observer=(0.0, 0.0))
    s.ignore(cone.id)
    _, pending = s.observe("robot_0", "pool_noodle", 9.0, 9.0, 0.4, observer=(0.0, 0.0))

    revived = ReviewStore(same_radius=0.5, ask_radius=1.5, ignore_radius=1.0)
    revived.load_dict(s.to_dict())

    assert revived.snapshot()["entities"] == s.snapshot()["entities"]
    assert revived.snapshot()["proposals"] == s.snapshot()["proposals"]
    assert len(revived.ignored) == 1
    assert pending.id in revived.proposals
    assert entity.id in revived.entities


def test_reload_keeps_the_centroids_weight_not_just_its_position():
    """Restoring only the averaged point would restart every object at weight 1.

    The next sighting after a restart would then yank a marker built from twenty
    viewpoints halfway towards itself.
    """
    s = store()
    _, first = s.observe("robot_0", "rubber_duck", 0.0, 0.0, 0.9, observer=(0.0, 0.0))
    entity = s.accept(first.id)
    for i in range(9):
        s.observe("robot_0", "rubber_duck", 0.0, 0.0, 0.9, observer=(float(i + 1), 0.0))
    assert entity.acc.count == 10

    revived = ReviewStore(same_radius=0.5, ask_radius=1.5)
    revived.load_dict(s.to_dict())
    survivor = revived.entities[entity.id]
    assert survivor.acc.count == 10

    # An eleventh viewpoint, badly wrong, must move it by a tenth of the error.
    revived.observe("robot_0", "rubber_duck", 0.44, 0.0, 0.9, observer=(99.0, 0.0))
    assert math.isclose(survivor.acc.x, 0.04, abs_tol=1e-9)


def test_reload_remembers_where_each_robot_was_standing():
    """Otherwise a robot that never moved reads as a fresh vantage point."""
    s = store()
    _, first = s.observe("robot_0", "rubber_duck", 0.0, 0.0, 0.9, observer=(2.0, 0.0))
    entity = s.accept(first.id)

    revived = ReviewStore(same_radius=0.5, ask_radius=1.5)
    revived.load_dict(s.to_dict())
    revived.observe("robot_0", "rubber_duck", 0.4, 0.0, 0.9, observer=(2.0, 0.0))

    survivor = revived.entities[entity.id]
    assert survivor.acc.count == 1, "a parked robot got a second vote after reload"
    assert survivor.acc.sightings == 2


def test_reload_never_reuses_an_id():
    """A stale dashboard must not be able to answer a different object."""
    s = store()
    _, first = s.observe("robot_0", "rubber_duck", 0.0, 0.0, 0.9)
    s.accept(first.id)
    _, pending = s.observe("robot_0", "rubber_duck", 8.0, 0.0, 0.9)

    revived = ReviewStore()
    # next_id deliberately corrupted backwards, as a hand-edited file might be.
    payload = s.to_dict()
    payload["next_id"] = 1
    revived.load_dict(payload)

    _, fresh = revived.observe("robot_0", "wooden_block", -8.0, 0.0, 0.9)
    assert fresh.id not in {first.id, pending.id}


def test_unreadable_state_does_not_take_the_store_with_it():
    s = store()
    for junk in (None, [], {"entities": "nope"}, {"entities": [{"no_id": 1}]},
                 {"ignored": [{"class": "x"}]}, {"entities": [{"id": "e", "count": 0}]}):
        s.load_dict(junk)
    assert not s.entities and not s.proposals

    # And a still-usable store afterwards.
    outcome, _ = s.observe("robot_0", "rubber_duck", 0.0, 0.0, 0.9)
    assert outcome == "proposed"


def test_forget_all_clears_confirmed_but_not_the_ignore_zones():
    s = store()
    _, duck = s.observe("robot_0", "rubber_duck", 0.0, 0.0, 0.9)
    s.accept(duck.id)
    _, block = s.observe("robot_0", "wooden_block", 9.0, 9.0, 0.9)
    s.ignore(block.id)

    assert s.forget_all() == 1
    assert not s.entities
    assert len(s.ignored) == 1, "ignoring is a separate decision from placing"
    assert s.forget_all() == 0
