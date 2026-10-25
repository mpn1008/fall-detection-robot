#!/usr/bin/env bash
set -euo pipefail
cd /workspaces/fall-detection-robot
mkdir -p ros_ws/src data models
rosdep update --rosdistro jazzy
echo 'ROS workspace ready. Optional ML dependencies: bash .devcontainer/scripts/setup-ml.sh cpu'
