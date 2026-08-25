"""adapter_sim must not inherit a hardware dashboard's robot_count."""


def test_cli_wins_over_env_and_yaml(sim_module):
    assert sim_module.resolve_sim_robot_count(2, "4", 5) == 2


def test_env_wins_over_yaml_when_cli_is_unset(sim_module):
    assert sim_module.resolve_sim_robot_count(None, "2", 7) == 2


def test_yaml_fleet_size_is_used_when_nothing_overrides_it(sim_module):
    assert sim_module.resolve_sim_robot_count(None, "", 2) == 2


def test_dashboard_seven_cannot_leak_in_as_the_default(sim_module):
    """settings.json robot_count=7 is a hardware session; sim defaults to 4."""
    assert sim_module.resolve_sim_robot_count(None, "", None) == 4


def test_count_is_clamped_to_the_spawnable_fleet(sim_module):
    assert sim_module.resolve_sim_robot_count(9, "", None) == 5
    assert sim_module.resolve_sim_robot_count(0, "", None) == 1
