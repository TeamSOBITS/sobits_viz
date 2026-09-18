<a name="readme-top"></a>

[JA](README_ja.md) | [EN](README.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBITS Viz

SOBITS Viz is a monorepo of visualizers for TeamSOBITS robots (SOBIT HOME,
SOBIT LIGHT, ...). Every visualizer shares one robot description: the
[sobits_vla_tools](https://github.com/TeamSOBITS/sobits_vla_tools)
`.robot.yaml` descriptor plus a per-robot views file. See
[CONTRIBUTING.md](CONTRIBUTING.md) for the conventions new code must follow.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Packages

| Package | Purpose | README |
| ------- | ------- | ------ |
| `sobits_viz_robots` | The robot descriptors every viewer shares | [README](sobits_viz_robots/README.md) |
| `sobits_viz_rerun` | Streams cameras, depth, laser scans, TF, joints, odometry and the robot model into the Rerun viewer | [README](sobits_viz_rerun/README.md) |
| `sobits_viz_foxglove` | Serves the same to the Foxglove viewer, with a layout generated per robot | [README](sobits_viz_foxglove/README.md) |

Planned: `sobits_viz_rviz` (shared RViz configs).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Getting started

### Prerequisites

| System | Version |
| ------ | ------- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS    | Jazzy Jalisco        |

A robot workspace with that robot's description package already built.

### Installation

```sh
cd ~/colcon_ws/src
git clone -b jazzy-devel https://github.com/TeamSOBITS/sobits_viz.git
cd sobits_viz
bash install.sh
cd ~/colcon_ws
rosdep install -i --from-paths src/sobits_viz -y
colcon build --symlink-install --packages-up-to sobits_viz
```

### Quickstart

After the robot's bringup:

```sh
ros2 launch sobits_viz_rerun rerun.launch.py robot_name:=sobit_home use_sim_time:=true
```

or, for SOBIT LIGHT:

```sh
ros2 launch sobits_viz_rerun rerun.launch.py robot_name:=sobit_light use_sim_time:=true
```

For Foxglove instead, start the bridge and open `ws://localhost:8765` in the
[desktop app](https://foxglove.dev/download) or at
[app.foxglove.dev](https://app.foxglove.dev), then import the layout it names:

```sh
ros2 launch sobits_viz_foxglove foxglove.launch.py robot_name:=sobit_home use_sim_time:=true
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Acknowledgments

- [Rerun](https://rerun.io) — Visualization SDK and viewer
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) — Robot middleware

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/sobits_viz.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/sobits_viz/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/sobits_viz.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/sobits_viz/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/sobits_viz.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/sobits_viz/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/sobits_viz.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/sobits_viz/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/sobits_viz.svg?style=for-the-badge
[license-url]: LICENSE
