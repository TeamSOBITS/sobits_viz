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
| SOBIT HOME | `sobit_home` | `ros2 launch sobit_home_bringup gz_minimal.launch.py headless:=true use_rviz:=false camera_rate:=30` | bare frame names |
| SOBIT LIGHT | `sobit_light` | `ros2 launch sobit_light_bringup gz_minimal.launch.py headless:=true use_rviz:=false enable_moveit:=false camera_rate:=30` | frames prefixed `sobit_light/` |

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
| `use_sim_time` | `false` | Set this to `true` in simulation |

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## The views file

`config/<robot>/<robot>.yaml` says what is shown and, because the bridge is
given exactly those topics, what is served. `views.scene` sets the fixed frame
and switches the model, frustums and scans; `views.cameras.<name>` names a
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

**Depth images.** Foxglove decodes `compressedDepth` only as 16-bit PNG, and
the simulated cameras publish `32FC1`, so the depth panels subscribe to the raw
image. On a real robot publishing 16UC1 you can set `use_compressed: true`.

**Commanded joints.** A `JointTrajectory` keeps its names in an array parallel
to the positions, which a message path cannot filter on, so a command series is
plotted by the joint's index in its descriptor group. Reordering a group's
joints in the descriptor changes which series is which.

**Sessions are not kept.** The bridge streams live data only; it records
nothing. Use Foxglove's own recording, or `ros2 bag`.

<p align="right">(<a href="#readme-top">back to top</a>)</p>
