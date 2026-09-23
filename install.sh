#!/bin/bash

set -e

echo "╔══╣ Install: SOBITS VIZ (STARTING) ╠══╗"

DIR=`pwd`
cd ..

# Everything from apt, including foxglove_bridge and rviz2, is declared in the
# packages' manifests and comes from rosdep.
rosdep update
rosdep install -r -y -i --from-paths ${DIR}

# The Rerun viewer is a pip package with no rosdep rule, and its version must
# match the C++ SDK sobits_viz_rerun is built against.
python3 -m pip install --break-system-packages rerun-sdk==0.37.2
rerun --version

cd ${DIR}

echo "╚══╣ Install: SOBITS VIZ (FINISHED) ╠══╝"
