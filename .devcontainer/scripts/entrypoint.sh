#!/usr/bin/env bash
set -euo pipefail

workspace=/workspaces/fall-detection-robot
# Fresh named volumes are populated from image directories. Also fix their
# ownership when VS Code updates the remote user's UID on a Linux host.
for directory in "$workspace/ros_ws/build" "$workspace/ros_ws/install" \
    "$workspace/ros_ws/log" "$HOME/.ros" "$HOME/.cache/pip"; do
    if [ ! -w "$directory" ]; then
        sudo chown -R "$(id -u):$(id -g)" "$directory"
    fi
done

python3 /usr/local/bin/make-dds-profile \
    --peers "${ROS_DDS_PEERS:-127.0.0.1}" \
    --address "${ROS_DDS_ADDRESS:-127.0.0.1}" \
    --output "${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds.xml}"

# ROS setup scripts are not guaranteed to be nounset-safe.
set +u
source /etc/profile.d/ros-dev.sh
set -u
exec "$@"
