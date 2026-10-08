# Copyright 2026 Team SOBITS
# SPDX-License-Identifier: BSD-3-Clause
"""Read a robot descriptor and a viewer parameter file, as both the launch and the generator do."""

from pathlib import Path

from sobits_robot_descriptor import DescriptorError, load_file, resolve_path
import yaml

KINDS = ('position', 'velocity', 'effort')


def find_descriptor(robot_id: str) -> str:
    """Resolve `<robot>.robot.yaml` through sobits_robot_descriptor."""
    try:
        return resolve_path(robot_id)
    except DescriptorError as error:
        raise SystemExit(str(error))


def load_params(path) -> dict:
    """Read the `ros__parameters` block out of a ROS parameter file."""
    with Path(path).open() as handle:
        document = yaml.safe_load(handle) or {}
    for node in document.values():
        if isinstance(node, dict) and 'ros__parameters' in node:
            return node['ros__parameters']
    raise SystemExit(f'{path} has no ros__parameters block')


def load_descriptor(path):
    """Read a schema v2 robot descriptor into a sobits_robot_descriptor.RobotDescriptor."""
    if not Path(path).is_file():
        raise SystemExit(
            f'{path} is not a file. The robot is described by a <robot>.robot.yaml, '
            'which sobits_robot_descriptor resolves; pass it with --descriptor.')
    try:
        return load_file(path)
    except DescriptorError as error:
        raise SystemExit(str(error))


def title(name: str) -> str:
    """`hand_left_camera` -> `Hand Left`."""
    return name.replace('_camera', '').replace('_', ' ').title()


def _topic(desc, rel):
    return desc.topic(rel) if rel else None


def stream_entry(desc, camera, stream) -> dict:
    """One camera stream with its topics made absolute."""
    return {
        'name': camera.name,
        'mount': camera.frame,
        'frame': stream.frame,
        'is_depth': stream.kind == 'depth',
        'raw_topic': _topic(desc, stream.raw_topic),
        'compressed_topic': _topic(desc, stream.compressed_topic),
        'info_topic': _topic(desc, stream.info_topic),
        'points_topic': _topic(desc, stream.points_topic),
        'range_m': stream.range_m,
    }


def relay_topics(params: dict) -> tuple:
    """Resolve the robot description topics the relay reads and writes."""
    robot = params.get('app_id', 'robot')
    relay = params.get('relay') or {}

    def resolved(key, fallback):
        topic = relay.get(key, fallback)
        return topic if topic.startswith('/') else f'/{robot}/{topic}'

    return resolved('input_topic', 'robot_description'), \
        resolved('output_topic', 'robot_description_foxglove')


def cameras(desc, settings: dict) -> list:
    """Camera streams the views file shows, colour then depth, as (name, label, entry, view)."""
    shown = []
    for camera in desc.cameras:
        view = settings.get(camera.name) or {}
        if not view.get('enable', True):
            continue
        label = view.get('name', title(camera.name))
        for stream in camera.streams():
            entry = stream_entry(desc, camera, stream)
            if entry['is_depth']:
                depth = view.get('depth') or {}
                if not depth.get('enable', True):
                    continue
                shown.append((camera.name, f'{label} depth', entry, depth))
            else:
                shown.append((camera.name, label, entry, view.get('color') or {}))
    return shown


def lidars(desc, settings: dict) -> list:
    """Laser scanners the views file shows, as (name, entry, view) tuples."""
    shown = []
    for lidar in desc.lidars:
        view = settings.get(lidar.name) or {}
        if not view.get('enable', True):
            continue
        entry = {'name': lidar.name, 'frame': lidar.frame,
                 'scan_topic': desc.topic(lidar.scan_topic),
                 'points_topic': _topic(desc, lidar.points_topic)}
        shown.append((lidar.name, entry, view))
    return shown


def joint_tabs(desc, settings: dict) -> list:
    """Plot tabs built from the descriptor's joint groups, as the Rerun layout does."""
    groups = {group.name: group for group in desc.groups}

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
            # Mimic joints are in joint_states but never in the group's command.
            joints += group.joints + group.uncommanded_joints
            if group.interface == 'trajectory' and group.command_topic:
                commanded.append((desc.topic(group.command_topic), list(group.joints)))
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


def bridged_topics(params: dict, desc) -> list:
    """Every topic the bridge should advertise for this robot's views."""
    views = params.get('views') or {}
    scene = views.get('scene') or {}
    topics = []

    if scene.get('enable', True):
        topics += ['/tf', '/tf_static']
        if scene.get('urdf', True):
            topics.append(relay_topics(params)[1])

    tabs = joint_tabs(desc, views.get('joints') or {})
    if tabs and desc.joint_states_topic:
        topics.append(desc.topic(desc.joint_states_topic))
    for tab in tabs:
        if tab['series']['command']:
            topics += [topic for topic, _ in tab['commands']]

    base = views.get('base') or {}
    mobile = desc.mobile_base
    if base.get('enable', True) and mobile:
        if mobile.odom_topic:
            topics.append(desc.topic(mobile.odom_topic))
        if base.get('command', False) and mobile.command_topic:
            topics.append(desc.topic(mobile.command_topic))

    for _, _, entry, view in cameras(desc, (views.get('cameras') or {})):
        compressed = view.get('use_compressed', not entry['is_depth'])
        image = entry['compressed_topic'] if compressed else entry['raw_topic']
        topics += [topic for topic in (image, entry['info_topic']) if topic]

    if scene.get('scans', True):
        topics += [entry['scan_topic'] for _, entry, _ in lidars(desc, views.get('lidars') or {})]

    return list(dict.fromkeys(topics))
