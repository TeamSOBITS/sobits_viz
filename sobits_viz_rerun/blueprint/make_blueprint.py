#!/usr/bin/env python3
"""Generate the default Rerun layout for the bridge.

Rerun decides on its own how to arrange entities when it has nothing else to
go on, and images that belong to a pinhole tend to end up inside the 3D view
rather than in panels of their own. This script writes a blueprint that gives
each camera its own 2D view next to the 3D scene, so the layout is the same
every time instead of depending on what the viewer guesses.

The cameras and joint groups come from the robot descriptor the node reads, and
the `views` block of the robot's parameter file names, enables or merges them.
Both live in config/<robot>/ with the layout this writes. Run it after changing
either:

    python3 blueprint/make_blueprint.py --params config/sobit_home/sobit_home.yaml
"""

from __future__ import annotations

import argparse
from pathlib import Path

import rerun.blueprint as rrb
import yaml

DEFAULT_PARAMS = (
    Path(__file__).resolve().parent.parent / "config" / "sobit_home" / "sobit_home.yaml"
)


def load_params(path: Path) -> dict:
    """Read the `ros__parameters` block out of a ROS parameter file."""
    with path.open() as handle:
        document = yaml.safe_load(handle)

    # The file is keyed by node name, usually the `/**` wildcard.
    for node in document.values():
        if isinstance(node, dict) and "ros__parameters" in node:
            return node["ros__parameters"]
    raise SystemExit(f"{path} has no ros__parameters block")


def load_descriptor(path: Path) -> dict:
    """Read a `sobits_vla_tools` robot descriptor."""
    with path.open() as handle:
        return yaml.safe_load(handle) or {}


def describe_robot(descriptor: Path) -> dict:
    """Load the robot descriptor, which is what names the cameras and groups."""
    if not descriptor.is_file():
        raise SystemExit(
            f"{descriptor} is not a file. The robot is described by a sobits_vla_tools "
            ".robot.yaml, which this needs to lay out its views; pass it with --descriptor."
        )
    return load_descriptor(descriptor)


def title(name: str) -> str:
    """`hand_left_camera` -> `Hand Left`."""
    return name.replace("_camera", "").replace("_", " ").title()


def camera_views(robot: dict, settings: dict) -> list[rrb.View]:
    """One 2D view per active camera in the descriptor, named as `settings` says."""
    views = []
    for entry in (robot.get("sensors") or {}).get("cameras") or []:
        if not entry.get("active", True):
            continue
        camera = entry["name"]
        view = settings.get(camera) or {}
        if not view.get("enable", True):
            continue
        name = view.get("name", title(camera))
        if entry.get("is_depth"):
            if not (view.get("depth") or {}).get("enable", True):
                continue
            views.append(
                rrb.Spatial2DView(name=f"{name} depth", origin=f"/cameras/{camera}/depth")
            )
        else:
            views.append(rrb.Spatial2DView(name=name, origin=f"/cameras/{camera}/color"))
    return views


def joint_tabs(robot: dict, settings: dict) -> list[rrb.View]:
    """Plot tabs built from the descriptor's joint groups.

    Every joint in one plot is unreadable, so each tab merges the groups it
    names; a group no tab names is not plotted. `**` matches whole path
    segments, not part of a name, so a tab lists its joints rather than
    matching a prefix like `arm_**`.
    """
    groups = {
        group["name"]: [joint["ros_name"] for joint in group.get("joints", [])]
        for group in robot.get("groups") or []
        if group.get("active", True)
    }

    # A tab says which series it plots; one that says nothing gets the position.
    kinds = ("position", "velocity", "effort")
    defaults = {kind: kind == "position" for kind in kinds}

    tabs: list[tuple[str, dict[str, bool], list[str]]] = []
    for key in settings.get("tabs") or []:
        tab = settings.get(key) or {}
        series = {kind: tab.get(kind, defaults[kind]) for kind in kinds}
        series["command"] = tab.get("command", False)
        joints: list[str] = []
        for group in tab.get("groups") or []:
            if group not in groups:
                raise SystemExit(f"joint tab {key!r} names unknown group {group!r}")
            joints += groups[group]
        joints += tab.get("add_joints") or []
        excluded = set(tab.get("exclude_joints") or [])
        joints = [joint for joint in dict.fromkeys(joints) if joint not in excluded]
        tabs.append((tab.get("name", key), series, joints))

    # A tab showing one series is a single plot; more are stacked in it. A
    # tab plotting commands puts them in the same plot as the observation.
    views: list[rrb.Container | rrb.View] = []
    for name, series, joints in tabs:
        shown = [kind for kind in kinds if series[kind]]
        plots = [
            rrb.TimeSeriesView(
                name=kind.title() if len(shown) > 1 else name,
                origin="/",
                contents=[f"/joints/{kind}/{joint}" for joint in joints] + (
                    [f"/joints/command/{kind}/{joint}" for joint in joints]
                    if series["command"] else []
                ),
            )
            for kind in shown
        ]
        if len(plots) == 1:
            views.append(plots[0])
        elif plots:
            views.append(rrb.Vertical(*plots, name=name))
    return views


def build(params: dict, descriptor: Path) -> rrb.Blueprint:
    robot = describe_robot(descriptor)
    robot_name = robot.get("robot_id", "robot")
    views = params.get("views") or {}

    # The whole tree: the robot model, which the URDF loader puts under its own
    # name at the root, the transforms, and the camera frustums. The view
    # resolves every pose into `target_frame`; left unset it would use the
    # origin entity's own frame, which the TF tree never connects to.
    scene_settings = views.get("scene") or {}
    contents = ["/**", "- /joints/**", "- /odom/**"]
    if not scene_settings.get("show_collision", False):
        contents.append(f"- /{robot_name}/collision_geometries/**")
    scene = rrb.Spatial3DView(
        name=scene_settings.get("name", "Scene"),
        origin="/",
        spatial_information=rrb.SpatialInformation(
            target_frame=scene_settings.get("target_frame", "odom")
        ),
        contents=contents,
    )

    plots = joint_tabs(robot, views.get("joints") or {})
    base_settings = views.get("base") or {}
    if base_settings.get("enable", True) and robot.get("mobile_base"):
        name = base_settings.get("name", "Base")
        kinds = [
            kind for kind in ("position", "velocity")
            if base_settings.get(kind, kind == "velocity")
        ]
        base = [
            rrb.TimeSeriesView(
                name=kind.title() if len(kinds) > 1 else name,
                origin="/",
                contents=[f"/odom/{kind}/**"] + (
                    ["/odom/command/**"]
                    if kind == "velocity" and base_settings.get("command", False) else []
                ),
            )
            for kind in kinds
        ]
        if len(base) == 1:
            plots.append(base[0])
        elif base:
            plots.append(rrb.Vertical(*base, name=name))

    left = []
    if scene_settings.get("enable", True):
        left.append(scene)
    if plots:
        left.append(rrb.Tabs(contents=plots))
    column = rrb.Vertical(*left, row_shares=[2, 1][: len(left)]) if left else None

    cameras = camera_views(robot, views.get("cameras") or {})

    columns = [c for c in (column, rrb.Grid(contents=cameras) if cameras else None) if c]
    if not columns:
        raise SystemExit("every view is disabled; nothing to lay out")
    return rrb.Blueprint(
        rrb.Horizontal(*columns, column_shares=[3, 2][: len(columns)]),
        collapse_panels=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--params", type=Path, default=DEFAULT_PARAMS)
    parser.add_argument(
        "--descriptor",
        type=Path,
        help="the sobits_vla_tools .robot.yaml; defaults to <app_id>.robot.yaml "
             "next to the parameter file, as the launch file's default does",
    )
    parser.add_argument(
        "--application-id",
        help="must match the bridge's app_id parameter; taken from the "
             "parameter file when not given",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="defaults to <app_id>.rbl next to the parameter file",
    )
    args = parser.parse_args()

    params = load_params(args.params)
    app_id = args.application_id or params.get("app_id", "robot")
    output = args.output or args.params.resolve().parent / f"{app_id}.rbl"
    descriptor = args.descriptor or args.params.resolve().parent / f"{app_id}.robot.yaml"

    output.parent.mkdir(parents=True, exist_ok=True)
    build(params, descriptor).save(app_id, output)
    print(f"wrote {output}")


if __name__ == "__main__":
    main()
