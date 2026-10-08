#!/bin/bash

set -e

echo "╔══╣ Install: SOBITS VIZ (STARTING) ╠══╗"

DIR=`pwd`
cd ..

# Clone sobits_robot_descriptor if missing
if [ ! -d "sobits_robot_descriptor" ]; then
    echo "Cloning: sobits_robot_descriptor"
    git clone --recurse-submodules -b $ROS_DISTRO-devel https://github.com/TeamSOBITS/sobits_robot_descriptor.git
fi

# Everything from apt, including foxglove_bridge and rviz2, is declared in the
# packages' manifests and comes from rosdep.
rosdep update
rosdep install -r -y -i --from-paths ${DIR} ${DIR}/../sobits_robot_descriptor

# The Rerun viewer is a pip package with no rosdep rule, and its version must
# match the C++ SDK sobits_viz_rerun is built against.
python3 -m pip install --break-system-packages rerun-sdk==0.37.2

cd ${DIR}

echo "╚══╣ Install: SOBITS VIZ (FINISHED) ╠══╝"
