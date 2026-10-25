#!/usr/bin/env bash
# Run inside the container. No simulator, camera or GPU required.
set -euo pipefail
set +u
source /etc/profile.d/ros-dev.sh
set -u

python -c 'import rclpy; import cv_bridge; print("ROS Python imports OK")'
python -B .devcontainer/scripts/test_dds_profile.py

log_directory=$(mktemp -d)
talker_pid=''
cleanup() {
    if [ -n "$talker_pid" ]; then
        kill "$talker_pid" 2>/dev/null || true
        wait "$talker_pid" 2>/dev/null || true
    fi
    rm -f "$log_directory/talker.log" "$log_directory/listener.log"
    rmdir "$log_directory"
}
trap cleanup EXIT

# Separate Python processes create separate DDS participants. A bounded timeout
# is expected for the listener, but success requires a received message.
python -c 'from demo_nodes_py.topics.talker import main; main()' >"$log_directory/talker.log" 2>&1 &
talker_pid=$!
status=0
timeout --signal=INT 12s python -c \
    'from demo_nodes_py.topics.listener import main; main()' \
    >"$log_directory/listener.log" 2>&1 || status=$?
if [ "$status" -ne 0 ] && [ "$status" -ne 124 ]; then
    cat "$log_directory/talker.log" "$log_directory/listener.log"
    exit "$status"
fi
if ! grep -q 'I heard' "$log_directory/listener.log"; then
    cat "$log_directory/talker.log" "$log_directory/listener.log"
    echo 'FAIL: no ROS message received through the UDP profile' >&2
    exit 1
fi
echo 'PASS: ROS talker/listener exchanged messages using the generated DDS profile'
