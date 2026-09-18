<a name="readme-top"></a>

[JA](README_ja.md) | [EN](README.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBITS Viz

SOBITS Vizは，TeamSOBITSのロボット（SOBIT HOME，SOBIT LIGHT，...）向けの
可視化ツールをまとめたモノレポです。すべての可視化ツールは1つのロボット
記述を共有します: [sobits_vla_tools](https://github.com/TeamSOBITS/sobits_vla_tools)
の`.robot.yaml`記述子と，ロボットごとのビュー設定ファイルです。新規コードが
従うべき規約は[CONTRIBUTING.md](CONTRIBUTING.md)を参照してください。

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## パッケージ

| パッケージ | 役割 | README |
| ------- | ------- | ------ |
| `sobits_viz_rerun` | カメラ，深度，レーザースキャン，TF，関節，オドメトリ，ロボットモデルをRerunビューアにストリーミング | [README](sobits_viz_rerun/README.md) |

予定: `sobits_viz_rviz`（共有RViz設定）と`sobits_viz_foxglove`（Foxglove
ブリッジとレイアウト）。

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## はじめに

### 前提条件

| システム | バージョン |
| ------ | ------- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS    | Jazzy Jalisco        |

ロボットの記述パッケージがビルド済みのロボットワークスペース。

### インストール

```sh
cd ~/colcon_ws/src
git clone -b jazzy-devel https://github.com/TeamSOBITS/sobits_viz.git
cd sobits_viz
bash install.sh
cd ~/colcon_ws
rosdep install -i --from-paths src/sobits_viz -y
colcon build --symlink-install --packages-up-to sobits_viz
```

### クイックスタート

ロボットのbringup後:

```sh
ros2 launch sobits_viz_rerun rerun.launch.py robot_name:=sobit_home use_sim_time:=true
```

SOBIT LIGHTの場合も同様です:

```sh
ros2 launch sobits_viz_rerun rerun.launch.py robot_name:=sobit_light use_sim_time:=true
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## 謝辞

- [Rerun](https://rerun.io) — 可視化SDKとビューア
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) — ロボットミドルウェア

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
