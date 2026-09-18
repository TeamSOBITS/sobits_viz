# Copyright 2026 Team SOBITS
# SPDX-License-Identifier: BSD-3-Clause
"""Tests for the generated Foxglove layouts."""

import json
from pathlib import Path

import pytest

from sobits_viz_foxglove.make_layout import build
from sobits_viz_foxglove.robot_views import bridged_topics, load_descriptor, load_params

PACKAGE = Path(__file__).resolve().parent.parent
ROBOTS = PACKAGE.parent / 'sobits_viz_robots' / 'config'
TYPES = ('3D', 'Image', 'Plot', 'Tab')


def robots():
    return sorted(p.name for p in (PACKAGE / 'config').iterdir()
                  if p.is_dir() and p.name != 'template')


def layout_of(robot):
    params = load_params(PACKAGE / 'config' / robot / f'{robot}.yaml')
    return params, build(params, ROBOTS / robot / f'{robot}.robot.yaml')


def leaves(node):
    if isinstance(node, str):
        yield node
    elif isinstance(node, dict):
        yield from leaves(node['first'])
        yield from leaves(node['second'])


@pytest.mark.parametrize('robot', robots())
def test_every_panel_is_placed_exactly_once(robot):
    _, layout = layout_of(robot)
    placed = list(leaves(layout['layout']))
    for tab in layout['configById'].get('Tab!plots', {}).get('tabs', []):
        placed += list(leaves(tab['layout']))
    assert sorted(placed) == sorted(layout['configById'])
    assert len(placed) == len(set(placed))


@pytest.mark.parametrize('robot', robots())
def test_panel_types_are_known(robot):
    _, layout = layout_of(robot)
    for panel_id in layout['configById']:
        assert panel_id.split('!')[0] in TYPES, panel_id


@pytest.mark.parametrize('robot', robots())
def test_every_plotted_topic_is_bridged(robot):
    params, layout = layout_of(robot)
    served = set(bridged_topics(params, load_descriptor(ROBOTS / robot / f'{robot}.robot.yaml')))
    for panel_id, config in layout['configById'].items():
        for path in config.get('paths', []):
            topic = path['value'].split('.')[0]
            assert topic in served, f'{panel_id} plots unserved {topic}'
        image = config.get('imageMode') or {}
        for key in ('imageTopic', 'calibrationTopic'):
            if image.get(key):
                assert image[key] in served, f'{panel_id} shows unserved {image[key]}'


@pytest.mark.parametrize('robot', robots())
def test_committed_layout_matches_the_generator(robot):
    _, layout = layout_of(robot)
    committed = (PACKAGE / 'config' / robot / f'{robot}.foxglove.json').read_text()
    assert committed == json.dumps(layout, indent=2, sort_keys=True) + '\n'
