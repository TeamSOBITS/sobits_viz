import os
import subprocess
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # A robot's parameters and layout live in config/<robot_name>/; its
        # descriptor in sobits_viz_robots. Each can be pointed elsewhere.
        DeclareLaunchArgument(
            'robot_name',
            description='Robot folder under config/ (sobit_home, sobit_light, ...)',
        ),
        DeclareLaunchArgument(
            'robot_descriptor', default_value='',
            description='The sobits_vla_tools .robot.yaml describing the robot to bridge; empty '
                        'means sobits_viz_robots config/<robot_name>/<robot_name>.robot.yaml',
        ),
        DeclareLaunchArgument(
            'robot_params', default_value='',
            description="The robot's app_id, sink and views; empty means "
                        'config/<robot_name>/<robot_name>.yaml',
        ),
        DeclareLaunchArgument(
            'output', default_value='',
            description='Where the generated layout is written; empty means a temporary file'),
        DeclareLaunchArgument(
            'blueprint', default_value='',
            description='A layout to use as it is; empty builds one from the views file',
        ),
        # The real robot's drivers stamp frames as "<robot_name>/<link>" while
        # the URDF uses bare link names; the prefix is stripped so they meet.
        DeclareLaunchArgument('enable_frame_prefix', default_value='true',
                              description='Strip frame_prefix from every frame id'),
        DeclareLaunchArgument('frame_prefix',    default_value='',
                              description='The prefix to strip; empty means "<robot_name>/"'),
        # The viewer is the launch file's: it starts the process the mode asks
        # for, so the mode and ports are arguments here and not in the file.
        DeclareLaunchArgument('viewer_mode',     default_value='spawn',
                              description='spawn | web | connect | serve'),
        DeclareLaunchArgument('grpc_port',       default_value='9876',
                              description='Port the bridge and viewer talk over'),
        DeclareLaunchArgument('connect_url',     default_value='rerun+http://127.0.0.1:9876/proxy',
                              description='Viewer to stream to in connect mode'),
        DeclareLaunchArgument('web_port',        default_value='9090',
                              description='Port the browser viewer is served on in web mode; '
                                          'its own gRPC port is grpc_port + 1'),
        # The streams and the views are the robot's
        # parameter file; the launch file adds only how this machine runs it.
        DeclareLaunchArgument('use_sim_time',    default_value='false'),
        # Wraps the bridge and the viewer, so they can be pinned together:
        # prefix:='taskset -c 0-3'.
        DeclareLaunchArgument('prefix',          default_value='',
                              description='Command the bridge and viewer run under'),
        DeclareLaunchArgument('robot_description_topic', default_value='robot_description',
                              description='Where robot_state_publisher latches the URDF; the one '
                                          'topic no descriptor names. Relative to /<robot_name>/, '
                                          'or absolute with a leading slash'),
        DeclareLaunchArgument('urdf_path', default_value='',
                              description='Where the robot description is written, for opening '
                                          'in a viewer by hand when embed_urdf is off'),
        DeclareLaunchArgument('memory_limit', default_value='10%',
                              description='Viewer memory ceiling; older data is dropped past it. '
                                          'The viewer holds decoded frames, far more than the '
                                          'wire carries, so a ceiling near total RAM starves '
                                          'the machine and the view falls behind.'),
        OpaqueFunction(function=launch_setup),
    ])


def _bool(lc, context):
    """Normalize CLI true/True/1 -> 'true', anything else -> 'false'."""
    return 'true' if lc.perform(context).lower() in ('true', '1', 'yes') else 'false'


def _generate(robot_name: str, params: str, descriptor: str, out_path: str) -> str:
    """Write the layout this robot's views file describes, and return its path."""
    script = os.path.join(get_package_share_directory('sobits_viz_rerun'),
                          'blueprint', 'make_blueprint.py')
    path = out_path or tempfile.mkstemp(prefix=f'{robot_name}_', suffix='.rbl')[1]
    if out_path:
        os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
    subprocess.run([sys.executable, script, '--params', params,
                    '--descriptor', descriptor, '--output', path], check=True)
    return path


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)

    config_dir = os.path.join(get_package_share_directory('sobits_viz_rerun'), 'config')
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
    # A robot's parts can be switched off at launch, so the layout is built from
    # the views file every time rather than read from one written earlier.
    blueprint = LaunchConfiguration('blueprint').perform(context)
    if not blueprint:
        blueprint = _generate(robot_name, robot_params, robot_descriptor,
                              LaunchConfiguration('output').perform(context).strip())
    viewer_mode = LaunchConfiguration('viewer_mode').perform(context)
    grpc_port = LaunchConfiguration('grpc_port').perform(context)
    connect_url = LaunchConfiguration('connect_url').perform(context)
    web_port = LaunchConfiguration('web_port').perform(context)

    actions = []
    urdf_path = LaunchConfiguration('urdf_path').perform(context) or \
        f'/tmp/sobits_viz_rerun_{robot_name}.urdf'

    # Two modes run the viewer as a separate process rather than letting the
    # SDK spawn one, so it can be handed the layout as a file. In both the
    # bridge hosts the stream and the viewer connects to it: a client sink drops
    # everything logged before a viewer accepts, while serving buffers it.
    prefix = LaunchConfiguration('prefix').perform(context).strip() or None
    node_mode = viewer_mode
    if viewer_mode == 'spawn':
        node_mode = 'serve'
        memory_limit = LaunchConfiguration('memory_limit').perform(context)
        actions.append(
            ExecuteProcess(
                cmd=[
                    # The ceiling matters on a laptop GPU: without it the viewer
                    # keeps every frame and eventually fails to allocate.
                    'rerun',
                    '--memory-limit', memory_limit,
                    blueprint,
                    '--connect', f'rerun+http://127.0.0.1:{grpc_port}/proxy',
                ],
                name='rerun_viewer',
                output='screen',
                prefix=prefix,
            )
        )
    elif viewer_mode == 'web':
        # The page is served from a viewer process that holds only the layout,
        # on the port above the bridge's; the browser streams from the bridge.
        # The process prints the URL to open, with both sources in it.
        node_mode = 'serve'
        actions.append(
            ExecuteProcess(
                cmd=[
                    'rerun',
                    '--serve-web',
                    '--web-viewer-port', web_port,
                    '--port', str(int(grpc_port) + 1),
                    blueprint,
                    f'rerun+http://127.0.0.1:{grpc_port}/proxy',
                ],
                name='rerun_web_viewer',
                output='screen',
                prefix=prefix,
            )
        )

    # What the node learns from the launch rather than from its file: the
    # robot's files, how this machine runs it, and the viewer it talks to.
    launch_params = {
        'robot_name': robot_name,
        'robot_descriptor': robot_descriptor,
        'urdf_path': urdf_path,
        'robot_description_topic': LaunchConfiguration('robot_description_topic').perform(context),
        'enable_frame_prefix':
            _bool(LaunchConfiguration('enable_frame_prefix'), context) == 'true',
        'frame_prefix': LaunchConfiguration('frame_prefix').perform(context),
        'use_sim_time': _bool(LaunchConfiguration('use_sim_time'), context) == 'true',
    }
    launch_params['viewer_mode'] = node_mode
    launch_params['grpc_port'] = int(grpc_port)
    launch_params['connect_url'] = connect_url

    bridge = Node(
        prefix=prefix,
        package='sobits_viz_rerun',
        executable='bridge',
        name='rerun_bridge',
        output='screen',
        parameters=[robot_params, launch_params],
    )
    actions.append(bridge)

    return actions
