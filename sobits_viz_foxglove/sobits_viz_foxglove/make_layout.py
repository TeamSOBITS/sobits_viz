# Copyright 2026 Team SOBITS
# SPDX-License-Identifier: BSD-3-Clause
"""
Generate the Foxglove layout a robot's views file describes.

Foxglove arranges nothing by itself: a fresh connection shows an empty
workspace. This writes the layout JSON to import once in the app, so every
robot opens with the same panels — the model and sensors in 3D, one image
panel per camera, and the joint plots grouped as the views file says.

The cameras, joint groups and lidars come from the robot descriptor in
sobits_viz_robots. Run it after changing either file:

    ros2 run sobits_viz_foxglove make_layout --params config/<robot>/<robot>.yaml
"""

import argparse
import json
from pathlib import Path

from sobits_viz_foxglove.robot_views import (
    bridged_topics,
    cameras,
    default_descriptor,
    joint_tabs,
    lidars,
    load_descriptor,
    load_params,
    relay_topics,
)

# Panels are stacked left of the camera grid; the 3D scene takes the most room.
_SCENE_SHARE = 66
_PLOTS_SHARE = 60


def _rgba(color: list, alpha: float = 1.0) -> str:
    red, green, blue = (color + [255, 255, 255])[:3]
    return f'rgba({red}, {green}, {blue}, {alpha})'


def _grid(ids: list) -> dict:
    """Arrange panel ids as a mosaic of rows, two per row."""
    if not ids:
        return None
    if len(ids) == 1:
        return ids[0]
    rows = [ids[i:i + 2] for i in range(0, len(ids), 2)]
    mosaic = None
    for index, row in enumerate(reversed(rows)):
        pair = row[0] if len(row) == 1 else {
            'direction': 'row',
            'first': row[0],
            'second': row[1],
            'splitPercentage': 50,
        }
        mosaic = pair if mosaic is None else {
            'direction': 'column',
            'first': pair,
            'second': mosaic,
            'splitPercentage': 100 / (index + 2),
        }
    return mosaic


def _scene_panel(params: dict, robot: dict, views: dict) -> dict:
    scene = views.get('scene') or {}
    prefix = params.get('frame_prefix', '')
    topics = {}

    for name, entry, view in lidars(robot, views.get('lidars') or {}):
        topics[entry['scan_topic']] = {
            'visible': scene.get('scans', True),
            'colorMode': 'flat',
            'flatColor': _rgba(view.get('color') or [255, 255, 255]),
            'pointSize': view.get('point_size_px', 4.0),
        }

    for name, label, entry, view in cameras(robot, views.get('cameras') or {}):
        camera = views.get('cameras', {}).get(name) or {}
        if entry.get('info_topic'):
            topics[entry['info_topic']] = {
                'visible': scene.get('frusta', True),
                'distance': camera.get('frustum_size_m', 0.3),
            }

    layers = {}
    if scene.get('urdf', True):
        layers['robot_model'] = {
            'instanceId': 'robot_model',
            'layerId': 'foxglove.Urdf',
            'label': params.get('app_id', 'robot'),
            'sourceType': 'topic',
            'topic': relay_topics(params)[1],
            'framePrefix': prefix,
            'displayMode': 'collision' if scene.get('show_collision') else 'visual',
            'visible': True,
            'order': 1,
        }

    return {
        'followTf': prefix + scene.get('fixed_frame', 'odom'),
        'followMode': 'follow-pose',
        'cameraState': {
            'distance': float(scene.get('camera_distance_m', 4.0)),
            'perspective': True, 'phi': 72.0, 'thetaOffset': 135.0,
            'targetOffset': [0.0, 0.0, float(scene.get('camera_height_m', 0.6))],
            'fovy': 45.0, 'near': 0.05, 'far': 200.0,
        },
        # Foxglove assumes meshes are Y-up; ROS meshes are Z-up.
        'scene': {'meshUpAxis': 'z_up', 'transforms': {'showLabel': False, 'axisScale': 0.0}},
        'topics': topics,
        'layers': layers,
        'imageMode': {},
    }


def _image_panels(params: dict, robot: dict, views: dict) -> tuple:
    panels, ids = {}, []
    for name, label, entry, view in cameras(robot, views.get('cameras') or {}):
        suffix = '_depth' if entry.get('is_depth') else ''
        panel_id = f'Image!{name}{suffix}'
        compressed = view.get('use_compressed', not entry.get('is_depth'))
        image = entry.get('compressed_topic') if compressed else entry.get('raw_topic')
        mode = {'imageTopic': image, 'calibrationTopic': entry.get('info_topic'),
                'synchronize': False}
        if entry.get('is_depth'):
            low, high = (view.get('range_m') or [0.0, 0.0])[:2]
            if high > low:
                mode['minValue'], mode['maxValue'] = low, high
        panels[panel_id] = {'imageMode': mode, 'foxglovePanelTitle': label}
        ids.append(panel_id)
    return panels, ids


def _joint_series(topic: str, kind: str, joints: list, order: list) -> list:
    # A JointState keeps its names in an array parallel to the values, which a
    # message path cannot filter on, so a joint is plotted by its index.
    series = []
    for joint in joints:
        if joint not in order:
            continue
        series.append({
            'value': f'{topic}.{kind}[{order.index(joint)}]',
            'label': joint,
            'enabled': True,
            'timestampMethod': 'receiveTime',
        })
    return series


def _command_series(commands: list, joints: list) -> list:
    # A JointTrajectory carries its names in a parallel array, which a message
    # path cannot filter on, so a command is plotted by its index in the group.
    series = []
    for topic, names in commands:
        for index, joint in enumerate(names):
            if joint not in joints:
                continue
            series.append({
                'value': f'{topic}.points[-1].positions[{index}]',
                'label': f'{joint} cmd',
                'enabled': True,
                'timestampMethod': 'receiveTime',
            })
    return series


def _plot(paths: list, title: str) -> dict:
    return {
        'paths': paths,
        'showLegend': True,
        'legendDisplay': 'floating',
        'showPlotValuesInLegend': False,
        'showXAxisLabels': True,
        'showYAxisLabels': True,
        'isSynced': True,
        'xAxisVal': 'timestamp',
        'followingViewWidth': 30,
        'sidebarDimension': 240,
        'foxglovePanelTitle': title,
    }


def _plot_panels(params: dict, robot: dict, views: dict) -> tuple:
    panels, tabs = {}, []
    states = robot.get('joint_states_topic', '/joint_states')
    order = params.get('joint_order') or []

    for tab in joint_tabs(robot, views.get('joints') or {}):
        shown = [kind for kind in ('position', 'velocity', 'effort') if tab['series'][kind]]
        ids = []
        for kind in shown:
            paths = _joint_series(states, kind, tab['joints'], order)
            if kind == 'position' and tab['series']['command']:
                paths += _command_series(tab['commands'], tab['joints'])
            panel_id = f"Plot!{tab['key']}_{kind}"
            title = tab['name'] if len(shown) == 1 else f"{tab['name']} {kind}"
            if not paths:
                continue
            panels[panel_id] = _plot(paths, title)
            ids.append(panel_id)
        if ids:
            tabs.append({'title': tab['name'], 'layout': _grid(ids) if len(ids) > 1 else ids[0]})

    base = views.get('base') or {}
    mobile = robot.get('mobile_base') or {}
    if base.get('enable', True) and mobile.get('odom_topic'):
        odom = mobile['odom_topic']
        paths = []
        if base.get('velocity', True):
            paths.append({'value': f'{odom}.twist.twist.linear.x', 'label': 'linear x (m/s)',
                          'enabled': True, 'timestampMethod': 'receiveTime'})
            if mobile.get('has_vel_y'):
                paths.append({'value': f'{odom}.twist.twist.linear.y', 'label': 'linear y (m/s)',
                              'enabled': True, 'timestampMethod': 'receiveTime'})
            paths.append({'value': f'{odom}.twist.twist.angular.z', 'label': 'yaw rate (rad/s)',
                          'enabled': True, 'timestampMethod': 'receiveTime'})
        if base.get('command', False) and mobile.get('command_topic'):
            command = mobile['command_topic']
            paths.append({'value': f'{command}.linear.x', 'label': 'cmd linear x (m/s)',
                          'enabled': True, 'timestampMethod': 'receiveTime'})
            paths.append({'value': f'{command}.angular.z', 'label': 'cmd yaw rate (rad/s)',
                          'enabled': True, 'timestampMethod': 'receiveTime'})
        if paths:
            name = base.get('name', 'Base')
            panels['Plot!base'] = _plot(paths, name)
            tabs.append({'title': name, 'layout': 'Plot!base'})

    return panels, tabs


def build(params: dict, descriptor) -> dict:
    """Return the Foxglove layout for this robot as a JSON-ready dict."""
    robot = load_descriptor(descriptor)
    views = params.get('views') or {}
    config = {}

    left = []
    if (views.get('scene') or {}).get('enable', True):
        config['3D!scene'] = _scene_panel(params, robot, views)
        left.append('3D!scene')

    plots, tabs = _plot_panels(params, robot, views)
    if tabs:
        config.update(plots)
        config['Tab!plots'] = {'activeTabIdx': 0, 'tabs': tabs}
        left.append('Tab!plots')

    images, image_ids = _image_panels(params, robot, views)
    config.update(images)

    column = left[0] if len(left) == 1 else {
        'direction': 'column',
        'first': left[0],
        'second': left[1],
        'splitPercentage': _SCENE_SHARE,
    } if left else None
    grid = _grid(image_ids)

    if column and grid:
        layout = {'direction': 'row', 'first': column, 'second': grid,
                  'splitPercentage': _PLOTS_SHARE}
    else:
        layout = column or grid
    if layout is None:
        raise SystemExit('every view is disabled; nothing to lay out')

    return {
        'configById': config,
        'globalVariables': {},
        'userNodes': {},
        'playbackConfig': {'speed': 1},
        'layout': layout,
    }


def main() -> None:
    """Write the layout of the robot the parameter file describes."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--params', type=Path, required=True,
        help="the robot's parameter file, config/<robot>/<robot>.yaml")
    parser.add_argument(
        '--descriptor', type=Path,
        help='the sobits_vla_tools .robot.yaml; defaults to the copy '
             'sobits_viz_robots keeps for <app_id>, as the launch file does')
    parser.add_argument(
        '--output', type=Path,
        help='defaults to <app_id>.foxglove.json next to the parameter file')
    args = parser.parse_args()

    params = load_params(args.params)
    app_id = params.get('app_id', 'robot')
    descriptor = args.descriptor or default_descriptor(app_id)
    output = args.output or args.params.resolve().parent / f'{app_id}.foxglove.json'

    output.parent.mkdir(parents=True, exist_ok=True)
    layout = build(params, descriptor)
    output.write_text(json.dumps(layout, indent=2, sort_keys=True) + '\n')
    topics = bridged_topics(params, load_descriptor(descriptor))
    print(f'wrote {output} ({len(layout["configById"])} panels, {len(topics)} topics)')


if __name__ == '__main__':
    main()
