# Copyright 2026 Team SOBITS
# SPDX-License-Identifier: BSD-3-Clause
"""Tests for the mesh URI rewrite."""

from sobits_viz_foxglove.mesh_uris import rewrite_file_uris


def test_installed_path_becomes_package_uri():
    urdf = '<mesh filename="file:///home/u/colcon_ws/install/kachaka_description' \
           '/share/kachaka_description/meshes/body.stl"/>'
    out, count = rewrite_file_uris(urdf)
    assert count == 1
    assert 'package://kachaka_description/meshes/body.stl' in out
    assert 'file://' not in out


def test_every_mesh_of_a_description_is_rewritten():
    urdf = ' '.join(
        f'<mesh filename="file:///opt/ros/jazzy/share/{pkg}/meshes/{pkg}.dae"/>'
        for pkg in ('sobit_light_description', 'realsense2_description')
    )
    out, count = rewrite_file_uris(urdf)
    assert count == 2
    assert out.count('package://') == 2


def test_package_uris_and_plain_paths_are_left_alone():
    urdf = '<mesh filename="package://sobit_home_description/meshes/base.stl"/>' \
           '<mesh filename="file:///tmp/scratch.stl"/>'
    out, count = rewrite_file_uris(urdf)
    assert count == 0
    assert out == urdf
