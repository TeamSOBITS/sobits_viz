<a name="readme-top"></a>

# sobits_viz_rerun

Streams the SOBIT HOME cameras, transforms and robot model into the
[Rerun](https://rerun.io) viewer.

Rerun does not speak ROS, so this package provides a node that subscribes to the
usual topics, converts each message and logs it to a Rerun stream. The result
is a single viewer showing the three camera streams as 2D images, the same
images projected into 3D as camera frustums, the depth cloud from the head
camera, the robot model posed by `/tf`, and every joint plotted over time.

| What | Where it ends up in Rerun |
| --- | --- |
| `head_camera` colour and depth | `cameras/head_camera/color`, `cameras/head_camera/depth` |
| `hand_left_camera` colour | `cameras/hand_left_camera/color` |
| `hand_right_camera` colour | `cameras/hand_right_camera/color` |
| `/tf`, `/tf_static` | `tf/<frame>`, `tf_static/<frame>` |
| descriptor `sensors.lidars` scans | `lidars/<name>/scan`, points in the scanner's frame |
| `joint_states` | `joints/position/<joint>`, `joints/velocity/<joint>`, `joints/effort/<joint>`, each only when a plot tab shows it |
| `odom` | `odom/velocity/linear_x`, `linear_y`, `speed`, `angular_z`; `odom/position/x`, `y`, `yaw` when the Base tab plots them |
| group `command_topic`s, `mobile_base.command_topic` | `joints/command/<series>/<joint>`, `odom/command/*`, when a tab plots commands |
| `robot_description` | `urdf/` |

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Requirements

The viewer binary comes from the Rerun Python package and has to match the SDK
version this node is built against (0.37.2). `sobit_home/install.sh` installs
it, or you can do it by hand:

```sh
$ python3 -m pip install --break-system-packages rerun-sdk==0.37.2
$ rerun --version    # expect rerun-cli 0.37.2
```

The C++ SDK itself is downloaded by CMake during the first build. That build
also compiles a minimal Apache Arrow from source and takes several minutes; any
later build reuses it. To build without network access, point CMake at a local
copy of the SDK archive:

```sh
$ colcon build --packages-select sobits_viz_rerun \
    --cmake-args -DRERUN_CPP_URL=/path/to/rerun_cpp_sdk.zip
```

> [!NOTE]
> If `lerobot` is installed in the same Python environment it pins an older
> `rerun-sdk`. Upgrading to 0.37.2 for the viewer will make pip report that
> conflict. This node does not use the Python package, but `lerobot`'s own Rerun
> logging may need a separate environment.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Launch and Usage

Start the robot first, in simulation or on the real hardware, then the bridge.

```sh
# Simulation
$ ros2 launch sobit_home_bringup gz_minimal.launch.py
$ ros2 launch sobits_viz_rerun rerun.launch.py use_sim_time:=true

# Real robot
$ ros2 launch sobit_home_bringup real_minimal.launch.py
$ ros2 launch sobits_viz_rerun rerun.launch.py
```

Where the viewer runs is a launch argument; what is shown is the robot's
parameter file, `config/<robot_name>/<robot_name>.yaml`.

### Where the viewer runs

| --- | --- |
| `spawn` (default) | Opens a viewer window next to the node. Needs a display. The bridge hosts the stream and the viewer connects back to it, so no data is lost while the viewer starts. |
| `web` | Serves the viewer as a web page on port 9090 and prints the URL to open; it carries the two sources the page must connect to, so open that URL and not the bare port. Works for a short look, not for watching. The browser viewer is 32-bit WebAssembly with a fixed 2.3 GiB ceiling that no argument raises ([rerun#11699](https://github.com/rerun-io/rerun/issues/11699)); at this data volume it reaches it within a minute or two, its garbage collector finds nothing to free, and the module crashes (`RuntimeError: unreachable`), which looks like a frozen page. Lower rates only delay it; Firefox grew past 20 GB instead. Use `spawn` on a display. Open the page soon after launching: the bridge replays everything it has buffered to a late viewer. |
| `connect` | Streams to a viewer that is already running, locally or on your laptop (`rerun --port 9876 sobit_home.rbl`, then `connect_url:=rerun+http://<viewer-host>:9876/proxy`). Start the viewer first: anything logged before it accepts the connection is discarded. |
| `serve` | Hosts the data in this process, buffered, for a viewer you start yourself — on the robot or on your laptop: `rerun --connect rerun+http://<robot>:9876/proxy sobit_home.rbl`. The `--connect` matters: `rerun <uri>` alone takes a listening bridge for a viewer and streams into it instead of opening a window. The remote option that works. |

```sh
# Browser viewer, useful over SSH; open the "connect at" URL it prints
$ ros2 launch sobits_viz_rerun rerun.launch.py viewer_mode:=web

# Viewer on your laptop, bridge on the robot
$ rerun --serve-web                      # on the laptop
$ ros2 launch sobits_viz_rerun rerun.launch.py \
      viewer_mode:=connect connect_url:=rerun+http://<laptop-ip>:9876/proxy
```

Nothing is written to disk: this package shows the robot live, it does not
archive it. Rerun's own viewer can save what it holds (File → Save) if a
snapshot is ever wanted.

### Arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `robot_name` | `sobit_home` | Names the robot's folder, `config/<robot_name>/`, and its topic namespace |
| `robot_descriptor` | `config/<robot_name>/<robot_name>.robot.yaml` | The `sobits_vla_tools` `.robot.yaml` describing the robot. Required. |
| `robot_params` | `config/<robot_name>/<robot_name>.yaml` | The robot's `app_id`, sink and `views` |
| `blueprint` | `config/<robot_name>/<robot_name>.rbl` | The viewer layout generated from the two above |
| `robot_description_topic` | `robot_description` | Where `robot_state_publisher` latches the URDF, under `/<robot_name>/` unless it starts with `/` |
| `enable_frame_prefix` | `true` | Strip `frame_prefix` from every frame id the drivers send |
| `frame_prefix` | `<robot_name>/` | The prefix to strip |
| `use_sim_time` | `false` | Set this to `true` in simulation |
| `viewer_mode` | `spawn` | See the table above |
| `grpc_port` | `9876` | Port the bridge and viewer talk over |
| `connect_url` | `rerun+http://127.0.0.1:9876/proxy` | Viewer to stream to in `connect` mode |
| `web_port` | `9090` | Port the browser viewer is served on; the page's own gRPC server, which holds the layout, takes `grpc_port + 1` |

The streams and the views are not launch arguments: they are the robot's
parameter file, `config/<robot_name>/<robot_name>.yaml`.
| `memory_limit` | `10%` | Viewer memory ceiling, as a size or a share of total RAM. The viewer holds decoded frames, far more than the wire carries, so a ceiling near total RAM starves the machine. |

### Describing another robot

The robot is described by a
[sobits_vla_tools](https://github.com/TeamSOBITS/sobits_vla_tools) descriptor,
which names every camera and topic. A copy for SOBIT HOME ships in
`config/sobit_home/`, next to the robot's own parameters and layout,
and is used by default; point the node at another robot's to bridge that one:

```sh
$ ros2 launch sobits_viz_rerun rerun.launch.py \
      robot_descriptor:=.../sobits_vla_common/robots/sobit_light.robot.yaml
```

Cameras marked `active: false` are skipped, and a camera with `is_depth: true`
is read as depth. Laser scanners come from a `sensors.lidars` list of
`{name, scan_topic, active}`, which the bundled copy adds to the schema.

A new robot starts from `config/template/`: copy it to `config/<robot>/`,
rename `template.robot.yaml` and `template.yaml` to `<robot>.robot.yaml` and
`<robot>.yaml`, fill in the `<...>` placeholders (or replace the descriptor
with the robot's own from `sobits_vla_common/robots/`), generate the layout
with `make_blueprint.py --params config/<robot>/<robot>.yaml`, and launch with
`robot_name:=<robot>`. There is no second description to fall back on: the node
stops if the descriptor is missing or unreadable.

Everything else a robot needs, its `app_id`, where the data goes and its
`views`, is in `config/<robot_name>/<robot_name>.yaml`. Among them, the cameras' `color.info_frame` and `depth.info_frame`
describe the robot rather than the viewer: a different
robot needs its own values for these, and nothing in the node assumes the
SOBIT HOME layout.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Viewer layout

Rerun arranges entities by itself when nothing tells it otherwise, and camera
images that belong to a pinhole tend to land inside the 3D view instead of
getting panels of their own. The package ships a blueprint that lays out the
3D scene above the joint plots, which are grouped into tabs by limb with the
base velocity alongside them, next to one 2D view per camera. The
collision geometry is left out of the 3D view, since its boxes cover the mesh
they approximate:

```sh
$ rerun $(ros2 pkg prefix sobits_viz_rerun)/share/sobits_viz_rerun/config/sobit_home/sobit_home.rbl
```

The layout follows the robot descriptor: every active camera gets a 2D view and
every joint group a plot tab, with the base velocity from `mobile_base`. The
`views` block of [config/sobit_home/sobit_home.yaml](config/sobit_home/sobit_home.yaml) adjusts that without
touching the descriptor: it renames a view, turns one off, sets the frame the
scene is resolved into, and for the joint tabs merges groups and adds or
excludes single joints. The node reads the same block: a view that is off is
not bridged either, and a camera's stream settings live in its entry, so each
camera is described in one place. A camera, or a setting, left out gets the
built-in defaults:

```yaml
views:
  scene: {name: "Scene", enable: true, target_frame: "odom", show_collision: false,
          embed_urdf: true, tf_rate_limit_hz: 20.0}
  cameras:
    head_camera:                         # its depth view is "<name> depth"
      name: "Head"
      frustum_size_m: 0.3
      color: {use_compressed: true, history: false, info_frame: "head_camera_color_optical_frame"}
      depth:                             # also datatype, colormap
        enable: true
        rate_limit_hz: 30.0
        history: false
        colormap_range_m: [0.1, 10.0]
        info_frame: "head_camera_depth_optical_frame"
  joints:
    rate_limit_hz: 30.0
    tabs: ["arms", "hands"]              # the tabs shown; a group no tab lists is not plotted
    arms:
      name: "Arms"
      groups: ["arm_left", "arm_right"]
      position: true                     # the series this tab plots; more than
      velocity: true                     # one are stacked. Only what some tab
      effort: false                      # plots is logged
      command: true                      # the same series from the groups' command topics
    hands:
      name: "Hands"
      groups: ["hand_left", "hand_right"]
      add_joints: ["hand_left_finger_r_mcp_joint"]   # in joint_states, not in a group
      exclude_joints: ["hand_right_finger_c_ip_joint"]
  lidars:
    lidar_front: {enable: true, rate_limit_hz: 10.0, point_radius_m: 0.02, color: [255, 96, 96]}
  base: {name: "Base", enable: true, position: false, velocity: true, command: true}
```

After changing the descriptor or `views`, regenerate the layout:

```sh
$ python3 blueprint/make_blueprint.py --params config/sobit_home/sobit_home.yaml
```

For another robot, point it at that robot's descriptor and parameter file. The
output name and the application id follow `app_id`:

```sh
$ python3 blueprint/make_blueprint.py --params /path/to/other_robot.yaml \
      --descriptor /path/to/other_robot.robot.yaml
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Notes

**Frame names.** The real wrist cameras stamp their images with frame ids like
`sobit_home/hand_left_camera_optical_frame`, while the URDF uses the bare
`hand_left_camera_optical_frame`. The bridge strips the robot name prefix from
every frame id it sees so that both spellings land on the same frame. Set
`enable_frame_prefix:=false` to turn that off, or `frame_prefix:=other/` to strip
something else.

**Depth units.** The head camera publishes millimetres as `16UC1` on the real
robot and metres as `32FC1` in simulation. Both are handled. Simulated depth
pixels beyond the far clip arrive as infinity and are logged as "no
measurement", otherwise they would stretch the viewer's depth range.

**Bandwidth.** Three raw colour streams at 30 Hz are roughly 80 MB/s, and about
1.5 GB per minute in the viewer, which is why `use_compressed` defaults to on.
Set it to `false` if you need the unaltered pixels.

**Colour lagging depth by seconds.** The SDK batches rows per entity and
sends an entity once its pending bytes reach `RERUN_FLUSH_NUM_BYTES`. At the
1 MiB default a depth frame ships within a few frames while ~4 KB JPEG rows
wait about ten seconds, and ~1 KB transform and joint rows arrive in lumps of
four to eight. The node sets `flush_num_bytes` (1000, below one row) before
opening the stream so every sample ships as logged; an already exported
`RERUN_FLUSH_NUM_BYTES` takes precedence for experiments.

**Sessions are not kept.** The viewer holds the session in memory only,
dropping the oldest data once its ceiling is reached and losing all of it when
the node exits. Streams logged with `history: false` are not even kept that
long: only their latest frame is in the viewer at any time.

**Missing covers on the model.** Rerun's COLLADA reader rejects the five `.dae`
meshes in `sobit_home_description` with `unexpected element library_nodes`, so
the cosmetic covers are absent while every `.stl` part loads. Converting those
meshes to STL is the workaround.

**Nothing shows up.** Check that the robot is publishing
(`ros2 topic hz /sobit_home/head_camera/color/image_raw`) and that the DDS
settings match; in the container that means
`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. If the robot model is missing
but the images are fine, the node logs a warning naming the first mesh file it
could not find, which usually means the workspace was not sourced.

<p align="right">(<a href="#readme-top">back to top</a>)</p>
