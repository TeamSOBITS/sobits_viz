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
| SOBIT HOME | `sobit_home` | `ros2 launch sobit_home_bringup gz_minimal.launch.py headless:=true use_rviz:=false camera_rate:=30` | bare frame names |
| SOBIT LIGHT | `sobit_light` | `ros2 launch sobit_light_bringup gz_minimal.launch.py headless:=true use_rviz:=false enable_moveit:=false camera_rate:=30` | frames prefixed `sobit_light/` |

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
| `config` | `config/<robot_name>/<robot_name>.rviz` | The RViz2 config `rviz2` opens with `-d` |
| `use_sim_time` | `false` | Set this to `true` in simulation |
| `enable_tf_prefix` | `false` | Frames are prefixed with `<robot_name>/`, matching the robot's own argument of the same name |

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## The views file

`config/<robot>/<robot>.yaml` says what the config displays.
`views.scene` sets the fixed frame, frames the opening view with
`camera_distance_m` and `camera_height_m`, and switches the grid, model, TF
tree and scans on or off; `views.tf` names the TF display and lists the
`frames` it draws and the ones to `exclude`, with `names` and `scale` sizing
the labels and axes; `views.cameras.<name>` names a camera's Image
display and picks compressed or raw for colour and depth, with `depth.name`
naming the depth display and `depth.points` holding the cloud's `name`,
`enable` and `size_px`;
`views.lidars.<name>` names a LaserScan display and sets its point size and
colour;
`views.base` adds an Odometry display. `frame_prefix` is prepended to the
fixed frame and given to the model's `TF Prefix`, for robots whose driver
prefixes its frames.

Regenerate the config after editing:

```sh
$ ros2 run sobits_viz_rviz make_config --params config/sobit_home/sobit_home.yaml
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
