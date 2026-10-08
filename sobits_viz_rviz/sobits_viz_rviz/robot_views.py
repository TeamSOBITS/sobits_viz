# Copyright 2026 Team SOBITS
# SPDX-License-Identifier: BSD-3-Clause
"""Read a robot descriptor and a viewer parameter file, as both the launch and the generator do."""

from pathlib import Path

from sobits_robot_descriptor import DescriptorError, load_file, resolve_path
import yaml


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
                shown.append((camera.name, depth.get('name', f'{label} depth'), entry, depth))
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
        shown.append((view.get('name', title(lidar.name)), entry, view))
    return shown


def tf_frames(desc) -> list:
    """Frames worth an axis by default: the base, the end effectors, the camera mounts."""
    frames = [desc.base_frame] + [e.ee_link for e in desc.ee] + [c.frame for c in desc.cameras]
    return list(dict.fromkeys(frames))
