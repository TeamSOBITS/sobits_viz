<a name="readme-top"></a>

# sobits_viz_rviz

Generate an RViz2 `.rviz` config for a TeamSOBITS robot: the model, TF tree
and laser scans in the 3D view, one Image display per camera, and odometry
for the base — arranged the same way for every robot the package supports.

The robot is described by the same
[sobits_vla_tools](https://github.com/TeamSOBITS/sobits_vla_tools) descriptor
the other viewers read, shared through the `sobits_viz_robots` package.

## Supported robots

| Robot | `robot_name` | Bringup for testing | Notes |
| --- | --- | --- | --- |
| SOBIT HOME | `sobit_home` | `ros2 launch sobit_home_bringup gz_minimal.launch.py headless:=true use_rviz:=false` | bare frame names |
| SOBIT LIGHT | `sobit_light` | `ros2 launch sobit_light_bringup gz_minimal.launch.py headless:=true use_rviz:=false enable_moveit:=false` | frames prefixed `sobit_light/` |

## Requirements

`rviz2` comes with the ROS 2 desktop install; nothing else is needed.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Usage

Start the robot, then RViz2 with the generated config:

```sh
$ ros2 launch sobits_viz_rviz rviz.launch.py robot_name:=sobit_home use_sim_time:=true
$ ros2 launch sobits_viz_rviz rviz.launch.py robot_name:=sobit_light use_sim_time:=true
```

### Arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `robot_name` | required | Robot folder under `config/`, and its topic namespace |
| `robot_params` | `config/<robot_name>/<robot_name>.yaml` | The robot's views |
| `robot_descriptor` | `sobits_viz_robots` `config/<robot_name>/<robot_name>.robot.yaml` | The descriptor naming the cameras and lidars |
| `config` | `''` | An RViz2 config to open as it is; the views file is not read and nothing is generated |
| `output` | `''` | Where the generated config is written; empty writes a temporary file |
| `use_sim_time` | `false` | Set this to `true` in simulation |
| `prefix` | `''` | Command the viewer runs under, e.g. `taskset -c 0-3` to pin it to those cores |
| `enable_tf_prefix` | `false` | Frames are prefixed with `<robot_name>/`, matching the robot's own argument of the same name |

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## The views file

The launch builds the config from this file every time, because a robot's parts can be
switched off at launch and a config written earlier would still show them. Nothing
is kept in the repository; `output` says where the generated config lands.

`config/<robot>/<robot>.yaml` says what the config displays. Every setting
below is optional; the default is what the table shows.

**`views.scene`** — the 3D view itself.

| Key | Default | Effect |
| --- | --- | --- |
| `enable` | `true` | `false` writes a config with no displays at all |
| `fixed_frame` | `odom` | The frame everything is drawn relative to |
| `camera_distance_m` | `4.0` | How far back the view opens |
| `camera_height_m` | `0.6` | What height it looks at |
| `grid` | `true` | The ground grid |
| `urdf` | `true` | The robot model |
| `show_collision` | `false` | Draw collision geometry instead of visual |

**`views.tf`** — the frame axes. RViz filters frames by regex, so `frames`
genuinely limits what is drawn.

| Key | Default | Effect |
| --- | --- | --- |
| `name` | `TF` | The display's name in the tree |
| `enable` | `false` | Ticked or not; the display is always listed |
| `label` | `false` | Draw each frame's name |
| `axis_scale` | `0.4` | Size of the axis markers |
| `frames` | all | Only these frames are drawn |
| `exclude` | none | These are not |

**`views.cameras.<name>`** — one Image display per camera, plus its depth
image and cloud.

| Key | Default | Effect |
| --- | --- | --- |
| `name` | from the descriptor | The panel's name |
| `enable` | `true` | Whether the camera appears at all |
| `use_compressed` | `true` for colour, `false` for depth | Subscribe to `/compressed` rather than the raw topic |
| `depth.name` | `<name> depth` | The depth panel's name |
| `depth.use_compressed` | `true` | `/compressedDepth` rather than raw |
| `depth.range_m` | unset | `[min, max]` metres across the greyscale; unset auto-normalizes |
| `depth.points.name` | `<name> points` | The cloud display's name |
| `depth.points.enable` | `false` | Ticked or not; always listed |
| `depth.points.size_px` | `2.0` | Point size |

**`views.lidars.<name>`** — one LaserScan display per scanner.

| Key | Default | Effect |
| --- | --- | --- |
| `name` | from the descriptor | The display's name |
| `enable` | `true` | Whether the scanner appears at all |
| `scan` | `true` | Ticked or not |
| `point_size_px` | `3.0` | Point size |
| `color` | white | `[r, g, b]`, 0–255 |

**`views.base`** adds an Odometry display, named by `name` and ticked by
`enable`.

The frame prefix is not in this file: the launch takes `enable_tf_prefix`
and prepends `<robot_name>/` to every frame, matching the robot's own
argument of the same name.

Editing this file is enough; the next launch picks it up. To write a config
without launching, for inspection or to hand to RViz yourself:

```sh
$ ros2 run sobits_viz_rviz make_config --params config/sobit_home/sobit_home.yaml \
    --output /tmp/sobit_home.rviz
```

A new robot takes the descriptor template in `sobits_viz_robots` and
`config/template/template.yaml` here.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Notes

**The `.rviz` format has no schema.** RViz2 silently ignores an unknown key
and silently drops a typo'd one — there is no validator to catch either. The
key spellings this generator writes are pinned from the `rviz_default_plugins`
and `rviz_common` Jazzy source, not guessed, and `Window Geometry` is left out
entirely: its `QMainWindow State` is an opaque Qt blob no generator can
synthesize, and RViz opens fine without one.

**Image transport is chosen by the topic suffix, not a key.** The Image
display has no transport property; `use_compressed` in the views file picks
between the raw topic and the one ending `/compressed` or `/compressedDepth`,
and RViz's `image_transport` plugin decodes it from that suffix alone.

<p align="right">(<a href="#readme-top">back to top</a>)</p>
