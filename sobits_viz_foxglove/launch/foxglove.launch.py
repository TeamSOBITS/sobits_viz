import json
import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sobits_viz_foxglove.make_layout import build
from sobits_viz_foxglove.robot_views import bridged_topics, load_descriptor, load_params


def generate_launch_description():
    return LaunchDescription([
        # A robot's views file and layout live in config/<robot_name>/; its
        # descriptor in sobits_viz_robots. Each can be pointed elsewhere.
        DeclareLaunchArgument(
            'robot_name',
            description='Robot folder under config/ (sobit_home, sobit_light, ...)',
        ),
        DeclareLaunchArgument(
            'robot_descriptor', default_value='',
            description='The sobits_vla_tools .robot.yaml describing the robot to serve; empty '
                        'means sobits_viz_robots config/<robot_name>/<robot_name>.robot.yaml',
        ),
        DeclareLaunchArgument(
            'robot_params', default_value='',
            description="The robot's views; empty means config/<robot_name>/<robot_name>.yaml",
        ),
        DeclareLaunchArgument(
            'layout', default_value='',
            description='The layout to import in the app; empty means '
                        'config/<robot_name>/<robot_name>.foxglove.json',
        ),
        # The viewer is a separate application: this only serves it.
        DeclareLaunchArgument('port', default_value='8765',
                              description='Port the Foxglove WebSocket listens on'),
        DeclareLaunchArgument('address', default_value='0.0.0.0',
                              description='Address the Foxglove WebSocket binds to'),
        DeclareLaunchArgument('video_transcode', default_value='false',
                              description='Let the bridge transcode images to video; off keeps '
                                          'the JPEG frames the cameras already publish'),
        DeclareLaunchArgument('robot_description_topic', default_value='robot_description',
                              description='Where robot_state_publisher latches the URDF. '
                                          'Relative to /<robot_name>/, or absolute with a slash'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # The robot's own flag, so the same value can be passed to both: with
        # it the driver stamps frames as "<robot_name>/<link>".
        DeclareLaunchArgument(
            'enable_tf_prefix', default_value='false',
            description='Frames are prefixed with <robot_name>/, as the robot does'),
        DeclareLaunchArgument(
            'tf_rate_hz', default_value='10.0',
            description='Rate /tf is republished at; 0 serves it untouched'),
        OpaqueFunction(function=launch_setup),
    ])


def _bool(lc, context):
    """Normalize CLI true/True/1 -> 'true', anything else -> 'false'."""
    return 'true' if lc.perform(context).lower() in ('true', '1', 'yes') else 'false'


def _prefixed(robot_name: str, params_path: str, descriptor: str) -> str:
    """Write a layout whose frames carry the robot's prefix, and return it."""
    params = load_params(params_path)
    params['frame_prefix'] = f'{robot_name}/'
    handle, path = tempfile.mkstemp(prefix=f'{robot_name}_', suffix='.foxglove.json')
    with os.fdopen(handle, 'w') as out:
        json.dump(build(params, descriptor), out, indent=2)
    return path


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)

    config_dir = os.path.join(get_package_share_directory('sobits_viz_foxglove'), 'config')
    robot_dir = os.path.join(config_dir, robot_name)
    robots_dir = os.path.join(get_package_share_directory('sobits_viz_robots'), 'config')

    def robot_file(arg, suffix):
        return LaunchConfiguration(arg).perform(context) or \
            os.path.join(robot_dir, f'{robot_name}{suffix}')

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
    robot_params = robot_file('robot_params', '.yaml')
    layout = robot_file('layout', '.foxglove.json')
    if _bool(LaunchConfiguration('enable_tf_prefix'), context) == 'true':
        layout = _prefixed(robot_name, robot_params, robot_descriptor)

    params = load_params(robot_params)
    topics = bridged_topics(params, load_descriptor(robot_descriptor))
    use_sim_time = _bool(LaunchConfiguration('use_sim_time'), context) == 'true'
    port = LaunchConfiguration('port').perform(context)

    # A viewer preloads every transform, so an idle robot's repeats fill its buffer.
    tf_rate = float(LaunchConfiguration('tf_rate_hz').perform(context))
    if tf_rate > 0:
        topics = ['/tf_throttled' if topic == '/tf' else topic for topic in topics]

    # Only what the layout shows is served; the bridge advertises everything by default.
    bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': int(port),
            'address': LaunchConfiguration('address').perform(context),
            'use_sim_time': use_sim_time,
            'topic_whitelist': ['^' + re.escape(topic) + '$' for topic in topics],
            'service_whitelist': ['(?!)'],
            'param_whitelist': ['(?!)'],
            'client_topic_whitelist': ['(?!)'],
            'video_transcode_topic_denylist':
                ['.*/compressedDepth']
                if _bool(LaunchConfiguration('video_transcode'), context) == 'true'
                else ['.*'],
            'sysinfo': False,
        }],
    )

    relay = Node(
        package='sobits_viz_foxglove',
        executable='description_relay',
        name='description_relay',
        output='screen',
        parameters=[robot_params, {
            'robot_name': robot_name,
            'input_topic': LaunchConfiguration('robot_description_topic').perform(context),
            'use_sim_time': use_sim_time,
        }],
    )

    throttle = Node(
        package='sobits_viz_foxglove',
        executable='transform_throttle',
        name='transform_throttle',
        output='screen',
        parameters=[{'rate_hz': tf_rate, 'use_sim_time': use_sim_time}],
    )

    return [
        bridge,
        relay,
        *([throttle] if tf_rate > 0 else []),
        LogInfo(msg=f'Foxglove: connect to ws://127.0.0.1:{port} and import {layout}'),
    ]
