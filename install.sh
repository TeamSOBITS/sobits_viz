#!/bin/bash

set -e

echo "╔══╣ Install: SOBITS VIZ (STARTING) ╠══╗"

# The Rerun viewer binary is the pip package, and it must match the C++ SDK
# version sobits_viz_rerun's bridge is built against.
python3 -m pip install --break-system-packages rerun-sdk==0.37.2
rerun --version

# The Foxglove bridge is an apt package; the viewer itself is the desktop app
# from https://foxglove.dev/download, or the web app.
if command -v apt-get > /dev/null; then
    sudo apt-get install -y "ros-${ROS_DISTRO}-foxglove-bridge"
fi

echo "╚══╣ Install: SOBITS VIZ (FINISHED) ╠══╝"
