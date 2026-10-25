#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
if [ -f /workspaces/fall-detection-robot/ros_ws/install/setup.bash ]; then
    source /workspaces/fall-detection-robot/ros_ws/install/setup.bash
fi
export VIRTUAL_ENV=/opt/venv
case ":$PATH:" in
    *:/opt/venv/bin:*) ;;
    *) export PATH="/opt/venv/bin:$PATH" ;;
esac
