"""Robot-local planner wiring must agree with each hardware adapter."""
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('robot,filename,domain', [
    ('botman', 'bunker', 17), ('aslan', 'aslan_bunker', 49),
    ('spot', 'spot', 0), ('asimov', 'unitree_g1', 0),
])
def test_onboard_planner_matches_adapter(robot, filename, domain):
    compose = yaml.safe_load((REPO / f'deploy/compose/docker-compose.robot-{robot}.yml').read_text())
    mgg = compose['services']['mgg']
    cfg = yaml.safe_load((REPO / f'adapters/adapter_ros2/config/{filename}.yaml').read_text())
    assert cfg['exploration']['enabled']
    assert mgg['network_mode'] == 'host'
    if robot == 'spot':
        assert mgg['environment']['ROS_LOCALHOST_ONLY'] == '1'
    assert 'profiles' not in mgg  # included by normal robot deployment
    assert mgg['environment']['SWARMDECK_ROBOT_ID'] == f'{robot}_0'
    assert mgg['environment']['SWARMDECK_ROBOT_CONFIG'].endswith(f'/{filename}.yaml')
    assert mgg['environment']['ROS_DOMAIN_ID'] == f'${{{robot.upper()}_ROS_DOMAIN_ID:-{domain}}}'
    planner = cfg['exploration']['planner']
    assert planner['cloud_topic'] != cfg['topics']['odom']
    if robot != 'asimov':
        assert planner['cloud_topic'] != cfg['topics']['map_cloud']
    assert planner['body_height_m'] > 0
    assert all(lo < hi for lo, hi in zip(planner['bounds_min'], planner['bounds_max']))
    assert cfg['actions'].get('navigate_to_pose') or cfg['actions'].get('trajectory')
