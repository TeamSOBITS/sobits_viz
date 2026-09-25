import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sobits_viz_rviz.make_config import _HEADER, build
from sobits_viz_rviz.robot_views import load_params

import yaml


def generate_launch_description():
    return LaunchDescription([
        # A robot's views file and config live in config/<robot_name>/; its
        # descriptor in sobits_viz_robots. Each can be pointed elsewhere.
        DeclareLaunchArgument(
            'robot_name',
            description='Robot folder under config/ (sobit_home, sobit_light, ...)',
        ),
        DeclareLaunchArgument(
            'robot_descriptor', default_value='',
            description='The sobits_vla_tools .robot.yaml describing the robot to show; empty '
                        'means sobits_viz_robots config/<robot_name>/<robot_name>.robot.yaml',
        ),
        DeclareLaunchArgument(
            'robot_params', default_value='',
            description="The robot's views; empty means config/<robot_name>/<robot_name>.yaml",
        ),
        DeclareLaunchArgument(
            'config', default_value='',
            description='The RViz2 config to open; empty means '
                        'config/<robot_name>/<robot_name>.rviz',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Wraps the viewer, so it can be pinned: prefix:='taskset -c 0-3'.
        DeclareLaunchArgument(
            'output', default_value='',
            description='Where the generated config is written; empty means a temporary file'),
        DeclareLaunchArgument('prefix', default_value='',
                              description='Command the viewer runs under, e.g. taskset'),
        # The robot's own flag, so the same value can be passed to both: with
        # it the driver stamps frames as "<robot_name>/<link>".
        DeclareLaunchArgument(
            'enable_tf_prefix', default_value='false',
            description='Frames are prefixed with <robot_name>/, as the robot does'),
        OpaqueFunction(function=launch_setup),
    ])


def _bool(lc, context):
    """Normalize CLI true/True/1 -> 'true', anything else -> 'false'."""
    return 'true' if lc.perform(context).lower() in ('true', '1', 'yes') else 'false'


def _generate(robot_name: str, params: dict, descriptor: str, out_path: str) -> str:
    """Write the config this robot's views file describes, and return its path."""
    config = build(params, descriptor)
    if out_path:
        os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
        handle = open(out_path, 'w')
        path = out_path
    else:
        fd, path = tempfile.mkstemp(prefix=f'{robot_name}_', suffix='.rviz')
        handle = os.fdopen(fd, 'w')
    with handle as out:
        out.write(_HEADER)
        yaml.safe_dump(config, out, default_flow_style=False, sort_keys=False)
    return path


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)

    config_dir = os.path.join(get_package_share_directory('sobits_viz_rviz'), 'config')
    robot_dir = os.path.join(config_dir, robot_name)
    robots_dir = os.path.join(get_package_share_directory('sobits_viz_robots'), 'config')

    robot_descriptor = LaunchConfiguration('robot_descriptor').perform(context) or \
        os.path.join(robots_dir, robot_name, f'{robot_name}.robot.yaml')
    if not os.path.isfile(robot_descriptor):
        robots = sorted(
            d for d in os.listdir(robots_dir)
            if d != 'template' and os.path.isdir(os.path.join(robots_dir, d))
        )
        raise RuntimeError(
            f'No robot descriptor at {robot_descriptor}. '
            f"Robots in sobits_viz_robots: {', '.join(robots)}")

    # A robot's parts can be switched off at launch, so the config is built from
    # the views file every time rather than read from one written earlier.
    config_path = LaunchConfiguration('config').perform(context)
    if not config_path:
        robot_params = LaunchConfiguration('robot_params').perform(context) or \
            os.path.join(robot_dir, f'{robot_name}.yaml')
        params = load_params(robot_params)
        if _bool(LaunchConfiguration('enable_tf_prefix'), context) == 'true':
            params['frame_prefix'] = f'{robot_name}/'
        config_path = _generate(robot_name, params, robot_descriptor,
                                LaunchConfiguration('output').perform(context).strip())
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() \
        in ('true', '1', 'yes')

    prefix = LaunchConfiguration('prefix').perform(context).strip()
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        prefix=prefix or None,
    )

    return [rviz]
