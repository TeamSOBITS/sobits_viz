<a name="readme-top"></a>

# sobits_viz_foxglove

Serve a TeamSOBITS robot to the [Foxglove](https://foxglove.dev) viewer: the
`foxglove_bridge` WebSocket with only the topics the robot's views file names,
a relay that makes the robot model's meshes fetchable, and a layout generated
per robot so the app opens with the panels already arranged.

The robot is described by the same
[sobits_vla_tools](https://github.com/TeamSOBITS/sobits_vla_tools) descriptor
the other viewers read, shared through the `sobits_viz_robots` package.

## Supported robots

| Robot | `robot_name` | Bringup for testing | Notes |
| --- | --- | --- | --- |
| SOBIT HOME | `sobit_home` | `ros2 launch sobit_home_bringup gz_minimal.launch.py headless:=true use_rviz:=false` | bare frame names |
| SOBIT LIGHT | `sobit_light` | `ros2 launch sobit_light_bringup gz_minimal.launch.py headless:=true use_rviz:=false enable_moveit:=false` | frames prefixed `sobit_light/` |

## Requirements

`foxglove_bridge` comes from apt, installed by the repository's `install.sh`:

```sh
$ sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

The viewer itself is a separate application. The desktop app is the supported
one: download the `.deb` from [foxglove.dev/download](https://foxglove.dev/download)
and install it with `sudo apt install ./foxglove-*.deb`. The web app at
[app.foxglove.dev](https://app.foxglove.dev) works too, from a Chromium-based
browser — Firefox and Safari block a `ws://` connection from an `https://` page.
Both need a Foxglove account; the free plan is enough.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Usage

Start the robot, then the bridge:

```sh
$ ros2 launch sobits_viz_foxglove foxglove.launch.py robot_name:=sobit_home use_sim_time:=true
$ ros2 launch sobits_viz_foxglove foxglove.launch.py robot_name:=sobit_light use_sim_time:=true
```

In the app: **Open connection** → **Foxglove WebSocket** → `ws://localhost:8765`
→ **Open**. Then **Layouts** → **Import from file…** and pick the robot's
layout, which the launch file prints on startup:

```
.../share/sobits_viz_foxglove/config/sobit_home/sobit_home.foxglove.json
```

The layout is imported once; the app remembers it.

### Arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `robot_name` | required | Robot folder under `config/`, and its topic namespace |
| `robot_params` | `config/<robot_name>/<robot_name>.yaml` | The robot's views |
| `robot_descriptor` | `sobits_viz_robots` `config/<robot_name>/<robot_name>.robot.yaml` | The descriptor naming the cameras, groups and lidars |
| `layout` | `config/<robot_name>/<robot_name>.foxglove.json` | The layout to import; only printed, never loaded by the node |
| `port` | `8765` | Port the Foxglove WebSocket listens on |
| `address` | `0.0.0.0` | Address it binds to |
| `video_transcode` | `false` | `true` lets the bridge re-encode images to video, which costs CPU and is unnecessary on a local network |
| `robot_description_topic` | `robot_description` | Where `robot_state_publisher` latches the URDF, under `/<robot_name>/` |
| `tf_rate_hz` | `10.0` | Rate `/tf` is republished at for the viewer; `0` serves it untouched |
| `use_sim_time` | `false` | Set this to `true` in simulation |
| `enable_tf_prefix` | `false` | Frames are prefixed with `<robot_name>/`, matching the robot's own argument of the same name |

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## The views file

`config/<robot>/<robot>.yaml` says what is shown and, because the bridge is
given exactly those topics, what is served. `views.scene` sets the fixed frame,
frames the opening view with `camera_distance_m` and `camera_height_m`, and
switches the model, frustums and scans; `views.tf` draws the frame axes with
`label`, `axis_scale`, `line_width` and `line_color`, and hides the frames
named in `exclude`, which is the only way to drop one: the panel shows every
frame it has not been told about, so on a robot with dozens the labels bury
the model and `enable` ships `false`; `views.cameras.<name>` names a
camera's panel and picks compressed or raw for colour and depth;
`views.lidars.<name>` sets point size and colour; `views.joints.tabs` lists the
plot tabs, each merging descriptor groups with optional `add_joints` and
`exclude_joints`; `views.base` plots the odometry twist. `frame_prefix` is
prepended to the fixed frame and given to the model layer, for robots whose
driver prefixes its frames.

Regenerate the layout after editing:

```sh
$ ros2 run sobits_viz_foxglove make_layout --params config/sobit_home/sobit_home.yaml
```

A new robot takes the descriptor template in `sobits_viz_robots` and
`config/template/template.yaml` here.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Notes

**Why the relay.** `xacro` expands `$(find pkg)` into an absolute install path,
so the description's meshes arrive as `file:///…/install/…`. Only a viewer
running on this machine could read those, and the browser never can. The relay
rewrites them to `package://<pkg>/…` and republishes the description, which the
bridge's asset service resolves for any client. `Failed to retrieve asset` in
the bridge log means the workspace holding those meshes was not sourced.

**Why `/tf` is throttled.** Foxglove preloads transforms and refuses more than
645277 per topic, then warns `Failed to process all transforms on topic /tf`.
SOBIT HOME publishes 42 frames at about 43 Hz, some 1800 transforms a second,
so that ceiling arrives in six minutes even standing still, where every repeat
carries the same pose. `transform_throttle` keeps the newest transform per
child frame and republishes them at `tf_rate_hz` on `/tf_throttled`, which the
bridge serves in place of `/tf`; motion still arrives, and the buffer lasts
about half an hour. Raise the rate for fast motion, or pass `tf_rate_hz:=0` to
serve `/tf` untouched.

**Camera frustums are off.** Foxglove draws the head colour frustum 90 degrees
out, pointing up rather than forward, from a `CameraInfo` indistinguishable
from the depth one beside it that draws correctly, on frames that differ only
by 24 mm of translation. The overlay carries no information the panels lack,
so `views.scene.frusta` ships `false`.

**Depth images.** Foxglove decodes `compressedDepth` only as 16-bit PNG, so the
depth panels subscribe to the raw image. The descriptors record the hardware
encoding, `16UC1`, and on a robot that publishes it you can set
`use_compressed: true`. A Gazebo depth sensor always emits `32FC1` whatever the
SDF asks for, which is twice the bytes: measured on SOBIT HOME, 1.23 MB a frame
against 0.92 MB for colour, enough that only 7 of 30 frames a second arrive and
their timestamps fall far enough behind for transform lookups to fail.

**Joints are plotted by index.** A `JointState` keeps its names in an array
parallel to the values, and a message path filter can only test fields of the
array it slices, so `position[:]{name=="…"}` matches nothing. Each series is
`position[<index>]` instead. The index cannot be trusted to a file, since a
publisher that reorders its joints would leave every plot reading the wrong
one without erroring, so the launch subscribes once and rebuilds the layout
with the live order. `joint_order` in the views file is the fallback for when
the robot is not running; capture it in wire order:

```sh
$ ros2 topic echo --once /<robot>/joint_states --field name
```

A joint missing from `joint_order` is skipped, so a passive mimic that no
publisher sends costs nothing. Commanded joints are indexed the same way,
within their descriptor group. Regenerate the layout whenever the robot's
joint set changes.

**Checking without the app.** `scripts/probe_bridge.py` connects the way a
viewer does and prints the channels the bridge advertises, and fetches one mesh
to prove the asset service and the relay work together:

```sh
$ python3 scripts/probe_bridge.py ws://127.0.0.1:8765 package://kachaka_description/meshes/kachaka/body.stl
capabilities: ['clientPublish', 'connectionGraph', 'parameters', ..., 'assets']
channels (21):
  /sobit_light/head_camera/color/image_raw/compressed | sensor_msgs/msg/CompressedImage
  ...
fetchAsset package://...: request 1, status 0, error '', 1439634 bytes
```

**Sessions are not kept.** The bridge streams live data only; it records
nothing. Use Foxglove's own recording, or `ros2 bag`.

<p align="right">(<a href="#readme-top">back to top</a>)</p>
