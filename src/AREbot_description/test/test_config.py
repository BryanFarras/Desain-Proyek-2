"""Smoke tests for the shipped YAML configuration files."""
from pathlib import Path

import pytest
import yaml

CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'
YAML_FILES = sorted(path.name for path in CONFIG_DIR.glob('*.yaml'))


def load(name):
    with open(CONFIG_DIR / name, 'r') as handle:
        return yaml.safe_load(handle)


@pytest.mark.parametrize('name', YAML_FILES)
def test_yaml_parses(name):
    data = load(name)
    assert isinstance(data, dict)
    assert data, f'{name} is empty'


def test_wheel_geometry_matches_bridge():
    wheel = load('wheel_odometry.yaml')['wheel_odometry']['ros__parameters']
    bridge = load('esp32_bridge.yaml')['esp32_bridge']['ros__parameters']
    for key in ('wheel_radius', 'wheel_separation'):
        assert wheel[key] == pytest.approx(bridge[key]), key


def test_mapper_mode_is_valid():
    params = load('mapper_params_online_async.yaml')['slam_toolbox'][
        'ros__parameters']
    assert params['mode'] in ('mapping', 'localization')


def test_nav2_has_expected_servers():
    params = load('nav2_navigation_params_real.yaml')
    for node in ('controller_server', 'planner_server', 'bt_navigator',
                 'behavior_server'):
        assert node in params, node


def test_default_map_is_configured():
    params = load('realbot_nav_setup.yaml')
    assert params['map_yaml'].endswith('.yaml')
    assert 'initial_pose' in params
