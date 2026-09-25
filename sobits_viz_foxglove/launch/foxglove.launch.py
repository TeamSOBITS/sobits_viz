import glob
import hashlib
import json
import os
import re
import tempfile
import time

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
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
        # Wraps every node this starts, so they can be pinned together:
        # prefix:='taskset -c 0-3'.
        DeclareLaunchArgument('prefix', default_value='',
                              description='Command the nodes run under, e.g. taskset'),
        # The bridge only serves data; this says who opens a window on it.
        DeclareLaunchArgument('viewer_mode', default_value='spawn',
                              description='spawn (the desktop app) | connect (serve only)'),
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


def _selected_layout_id(store: str) -> str:
    """Read the layout id the app will open from its Local Storage."""
    # The app reopens whatever it had selected, so writing a new layout beside
    # it changes nothing until that id is the one on disk.
    leveldb = os.path.expanduser('~/.config/Foxglove/Local Storage/leveldb')
    found = ''
    for name in sorted(os.listdir(leveldb)) if os.path.isdir(leveldb) else []:
        try:
            blob = open(os.path.join(leveldb, name), 'rb').read()
        except OSError:
            continue
        for match in re.finditer(rb'"currentLayoutId":"(lay_[A-Za-z0-9]+)"', blob):
            found = match.group(1).decode()
    return found


def _install_layout(robot_name: str, layout: str) -> None:
    """Put the layout where the app reads it, so panels exist on connect."""
    # Connecting alone shows an empty session: with no panels, nothing
    # subscribes and the bridge serves only the latched description.
    store = os.path.expanduser('~/.config/Foxglove/studio-datastores')
    if not os.path.isdir(store) or not os.path.isfile(layout):
        return
    chosen = _selected_layout_id(store)
    target = os.path.join(store, 'layouts-local')
    for existing in sorted(glob.glob(os.path.join(store, 'layouts-*'))):
        if chosen and os.path.isfile(os.path.join(existing, chosen)):
            target = existing
            break
    os.makedirs(target, exist_ok=True)
    saved = time.strftime('%Y-%m-%dT%H:%M:%S.000Z', time.gmtime())
    # The app wants lay_ and 16 alphanumerics; anything else it tries to sync
    # and the server rejects as an invalid id.
    digest = hashlib.sha1(robot_name.encode()).hexdigest()[:16]
    entry_id = chosen or f'lay_{digest}'
    entry = {
        'id': entry_id,
        'name': f'{robot_name} (sobits_viz)',
        'permission': 'CREATOR_WRITE',
        'baseline': {'data': json.load(open(layout)), 'savedAt': saved},
        'working': None,
    }
    with open(os.path.join(target, entry_id), 'w') as out:
        json.dump(entry, out)


def _live_joint_order(topic: str, timeout: float = 5.0) -> list:
    """Read the joint names from one JointState message, in wire order."""
    # A plot addresses a joint by its index in the message, so the order has to
    # come from the robot: a stale copy plots the wrong joint without erroring.
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except ImportError:
        return []
    seen = []
    started = rclpy.ok()
    if not started:
        rclpy.init()
    node = Node('foxglove_joint_order')
    node.create_subscription(JointState, topic, lambda m: seen.append(list(m.name)), 10)
    deadline = time.time() + timeout
    while not seen and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    if not started:
        rclpy.shutdown()
    return seen[0] if seen else []


def _regenerated(robot_name: str, params: dict, descriptor: str) -> str:
    """Write a layout built from these params, and return its path."""
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

    params = load_params(robot_params)
    robot = load_descriptor(robot_descriptor)
    stale = False
    if _bool(LaunchConfiguration('enable_tf_prefix'), context) == 'true':
        params['frame_prefix'] = f'{robot_name}/'
        stale = True
    # A plot reads a joint by index, so the order has to match the robot that
    # is running, not whatever the file was written against.
    live = _live_joint_order(robot.get('joint_states_topic', '/joint_states'))
    if live and live != (params.get('joint_order') or []):
        params['joint_order'] = live
        stale = True
    if stale:
        layout = _regenerated(robot_name, params, robot_descriptor)

    topics = bridged_topics(params, robot)
    use_sim_time = _bool(LaunchConfiguration('use_sim_time'), context) == 'true'
    port = LaunchConfiguration('port').perform(context)

    # A viewer preloads every transform, so an idle robot's repeats fill its buffer.
    tf_rate = float(LaunchConfiguration('tf_rate_hz').perform(context))
    if tf_rate > 0:
        topics = ['/tf_throttled' if topic == '/tf' else topic for topic in topics]

    # Only what the layout shows is served; the bridge advertises everything by default.
    prefix = LaunchConfiguration('prefix').perform(context).strip() or None
    bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        prefix=prefix,
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
        prefix=prefix,
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
        prefix=prefix,
        output='screen',
        parameters=[{'rate_hz': tf_rate, 'use_sim_time': use_sim_time}],
    )

    mode = LaunchConfiguration('viewer_mode').perform(context).strip().lower()
    if mode not in ('spawn', 'connect'):
        raise RuntimeError(f"viewer_mode must be spawn or connect, not '{mode}'")

    started = []
    if mode == 'spawn':
        _install_layout(robot_name, layout)
        started.append(ExecuteProcess(
            cmd=['foxglove-studio', '--no-sandbox',
                 f'foxglove://open?ds=foxglove-websocket&ds.url=ws://127.0.0.1:{port}'],
            output='log',
            prefix=prefix,
        ))

    return [
        bridge,
        relay,
        *([throttle] if tf_rate > 0 else []),
        *started,
        LogInfo(msg=f'Foxglove: connect to ws://127.0.0.1:{port} and import {layout}'),
    ]
