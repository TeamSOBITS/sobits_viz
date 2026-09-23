# Copyright 2026 Team SOBITS
# SPDX-License-Identifier: BSD-3-Clause
"""Read a robot descriptor and a viewer parameter file, as both the launch and the generator do."""

from pathlib import Path

import yaml


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
            shown.append((camera, depth.get('name', f'{label} depth'), entry, depth))
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
