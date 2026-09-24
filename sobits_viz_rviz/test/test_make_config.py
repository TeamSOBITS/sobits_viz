# Copyright 2026 Team SOBITS
# SPDX-License-Identifier: BSD-3-Clause
"""Tests for the generated RViz2 configs."""

from pathlib import Path

import pytest
from sobits_viz_rviz.make_config import _HEADER, build
from sobits_viz_rviz.robot_views import load_params
import yaml

PACKAGE = Path(__file__).resolve().parent.parent
ROBOTS = PACKAGE.parent / 'sobits_viz_robots' / 'config'
TOP_LEVEL_KEYS = {'Panels', 'Visualization Manager', 'Window Geometry'}
FORBIDDEN_KEYS = {'Links', 'Frames', 'Tree', 'Namespaces'}


def robots():
    return sorted(p.name for p in (PACKAGE / 'config').iterdir()
                  if p.is_dir() and p.name != 'template')


def config_of(robot):
    params = load_params(PACKAGE / 'config' / robot / f'{robot}.yaml')
    return params, build(params, ROBOTS / robot / f'{robot}.robot.yaml')


def walk(node):
    """Yield every (key, value) pair anywhere in a nested dict/list."""
    if isinstance(node, dict):
        for key, value in node.items():
            yield key, value
            yield from walk(value)
    elif isinstance(node, list):
        for item in node:
            yield from walk(item)


@pytest.mark.parametrize('robot', robots())
def test_top_level_keys_are_known(robot):
    _, config = config_of(robot)
    assert set(config) <= TOP_LEVEL_KEYS


@pytest.mark.parametrize('robot', robots())
def test_window_names_every_image_panel(robot):
    _, config = config_of(robot)
    geometry = config['Window Geometry']
    for display in config['Visualization Manager']['Displays']:
        if display['Class'].endswith('/Image'):
            assert display['Name'] in geometry
    state = geometry.get('QMainWindow State')
    if state:
        assert bytes.fromhex(state)[:4] == b'\x00\x00\x00\xff'


@pytest.mark.parametrize('robot', robots())
def test_every_display_has_the_required_keys(robot):
    _, config = config_of(robot)
    for display in config['Visualization Manager']['Displays']:
        for key in ('Class', 'Name', 'Enabled', 'Value'):
            assert key in display, f'{display.get("Name", display)} missing {key}'
        assert display['Enabled'] == display['Value']


@pytest.mark.parametrize('robot', robots())
def test_display_classes_are_prefixed_correctly(robot):
    _, config = config_of(robot)
    for display in config['Visualization Manager']['Displays']:
        assert display['Class'].startswith('rviz_default_plugins/')
    for tool in config['Visualization Manager']['Tools']:
        assert tool['Class'].startswith('rviz_default_plugins/')


@pytest.mark.parametrize('robot', robots())
def test_panel_classes_are_prefixed_correctly(robot):
    _, config = config_of(robot)
    for panel in config['Panels']:
        assert panel['Class'].startswith('rviz_common/')


@pytest.mark.parametrize('robot', robots())
def test_no_runtime_derived_keys(robot):
    _, config = config_of(robot)
    found = {key for key, _ in walk(config) if key in FORBIDDEN_KEYS}
    assert not found, f'runtime-derived keys leaked into the config: {found}'


@pytest.mark.parametrize('robot', robots())
def test_committed_config_matches_the_generator(robot):
    _, config = config_of(robot)
    committed = (PACKAGE / 'config' / robot / f'{robot}.rviz').read_text()
    expected = _HEADER + yaml.safe_dump(config, default_flow_style=False, sort_keys=False)
    assert committed == expected


@pytest.mark.parametrize('robot', robots())
def test_fixed_frame_carries_the_frame_prefix(robot):
    # The prefix is empty in the file and set by the launch, so the test drives
    # it rather than asserting what a robot happens to ship with.
    params, config = config_of(robot)
    fixed_frame = config['Visualization Manager']['Global Options']['Fixed Frame']
    assert fixed_frame == params.get('frame_prefix', '') + params['views']['scene']['fixed_frame']
    params['frame_prefix'] = 'prefixed/'
    prefixed = build(params, ROBOTS / robot / f'{robot}.robot.yaml')
    assert prefixed['Visualization Manager']['Global Options']['Fixed Frame'].startswith(
        'prefixed/')
