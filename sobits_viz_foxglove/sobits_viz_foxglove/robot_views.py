# Copyright 2026 Team SOBITS
# SPDX-License-Identifier: BSD-3-Clause
"""Read a robot descriptor and a viewer parameter file, as both the launch and the generator do."""

from pathlib import Path

import yaml

KINDS = ('position', 'velocity', 'effort')


def default_descriptor(app_id: str) -> Path:
    """Where sobits_viz_robots keeps this robot's descriptor."""
    try:
        from ament_index_python.packages import get_package_share_directory
        config = Path(get_package_share_directory('sobits_viz_robots')) / 'config'
    except Exception:
        # Running from the source tree, before the workspace is built.
        config = Path(__file__).resolve().parents[3] / 'sobits_viz_robots' / 'config'
    return config / app_id / f'{app_id}.robot.yaml'


def load_params(path) -> dict:
    """Read the `ros__parameters` block out of a ROS parameter file."""
    with Path(path).open() as handle:
        document = yaml.safe_load(handle) or {}
    for node in document.values():
        if isinstance(node, dict) and 'ros__parameters' in node:
            return node['ros__parameters']
    raise SystemExit(f'{path} has no ros__parameters block')


def load_descriptor(path) -> dict:
    """Read a sobits_vla_tools robot descriptor."""
    path = Path(path)
    if not path.is_file():
        raise SystemExit(
            f'{path} is not a file. The robot is described by a sobits_vla_tools '
            '.robot.yaml, which sobits_viz_robots keeps; pass it with --descriptor.')
    with path.open() as handle:
        return yaml.safe_load(handle) or {}


def title(name: str) -> str:
    """`hand_left_camera` -> `Hand Left`."""
    return name.replace('_camera', '').replace('_', ' ').title()


def relay_topics(params: dict) -> tuple:
    """Resolve the robot description topics the relay reads and writes."""
    robot = params.get('app_id', 'robot')
    relay = params.get('relay') or {}

    def resolved(key, fallback):
        topic = relay.get(key, fallback)
        return topic if topic.startswith('/') else f'/{robot}/{topic}'

    return resolved('input_topic', 'robot_description'), \
        resolved('output_topic', 'robot_description_foxglove')


def cameras(robot: dict, settings: dict) -> list:
    """Active cameras the views file shows, as (name, label, entry, view) tuples."""
    shown = []
    for entry in (robot.get('sensors') or {}).get('cameras') or []:
        if not entry.get('active', True):
            continue
        camera = entry['name']
        view = settings.get(camera) or {}
        if not view.get('enable', True):
            continue
        label = view.get('name', title(camera))
        if entry.get('is_depth'):
            depth = view.get('depth') or {}
            if not depth.get('enable', True):
                continue
            shown.append((camera, f'{label} depth', entry, depth))
        else:
            shown.append((camera, label, entry, view.get('color') or {}))
    return shown


def lidars(robot: dict, settings: dict) -> list:
    """Active laser scanners the views file shows, as (name, entry, view) tuples."""
    shown = []
    for entry in (robot.get('sensors') or {}).get('lidars') or []:
        if not entry.get('active', True):
            continue
        view = settings.get(entry['name']) or {}
        if not view.get('enable', True):
            continue
        shown.append((entry['name'], entry, view))
    return shown


def joint_tabs(robot: dict, settings: dict) -> list:
    """Plot tabs built from the descriptor's joint groups, as the Rerun layout does."""
    groups = {
        group['name']: group
        for group in robot.get('groups') or []
        if group.get('active', True)
    }

    # A tab says which series it plots; one that says nothing gets the position.
    defaults = {kind: kind == 'position' for kind in KINDS}

    tabs = []
    for key in settings.get('tabs') or []:
        tab = settings.get(key) or {}
        series = {kind: tab.get(kind, defaults[kind]) for kind in KINDS}
        series['command'] = tab.get('command', False)
        joints, commanded = [], []
        for name in tab.get('groups') or []:
            if name not in groups:
                raise SystemExit(f'joint tab {key!r} names unknown group {name!r}')
            group = groups[name]
            names = [joint['ros_name'] for joint in group.get('joints') or []]
            joints += names
            if group.get('command_topic'):
                commanded.append((group['command_topic'], names))
        joints += tab.get('add_joints') or []
        excluded = set(tab.get('exclude_joints') or [])
        joints = [joint for joint in dict.fromkeys(joints) if joint not in excluded]
        tabs.append({
            'key': key,
            'name': tab.get('name', title(key)),
            'series': series,
            'joints': joints,
            'commands': commanded,
        })
    return tabs


def bridged_topics(params: dict, robot: dict) -> list:
    """Every topic the bridge should advertise for this robot's views."""
    views = params.get('views') or {}
    scene = views.get('scene') or {}
    topics = []

    if scene.get('enable', True):
        topics += ['/tf', '/tf_static']
        if scene.get('urdf', True):
            topics.append(relay_topics(params)[1])

    tabs = joint_tabs(robot, views.get('joints') or {})
    if tabs and robot.get('joint_states_topic'):
        topics.append(robot['joint_states_topic'])
    for tab in tabs:
        if tab['series']['command']:
            topics += [topic for topic, _ in tab['commands']]

    base = views.get('base') or {}
    mobile = robot.get('mobile_base') or {}
    if base.get('enable', True) and mobile:
        if mobile.get('odom_topic'):
            topics.append(mobile['odom_topic'])
        if base.get('command', False) and mobile.get('command_topic'):
            topics.append(mobile['command_topic'])

    for _, _, entry, view in cameras(robot, (views.get('cameras') or {})):
        compressed = view.get('use_compressed', not entry.get('is_depth'))
        image = entry.get('compressed_topic') if compressed else entry.get('raw_topic')
        topics += [topic for topic in (image, entry.get('info_topic')) if topic]

    if scene.get('scans', True):
        topics += [entry['scan_topic'] for _, entry, _ in lidars(robot, views.get('lidars') or {})]

    return list(dict.fromkeys(topics))
