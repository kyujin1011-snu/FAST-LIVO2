#!/bin/bash
set -e

# Always unset to avoid collision
unset ROS_DISTRO

# Source ROS 2 (Foxy) first
source /opt/ros/foxy/setup.bash
source /opt/ros/foxy/setup.bash

unset ROS_DISTRO
# Source ROS 1 (Noetic)
source /opt/ros/noetic/setup.bash
source /opt/ros/noetic/setup.bash

unset ROS_DISTRO
# Source overlay workspace
if [ -f /root/ros_ws/install/setup.bash ]; then
    source /root/ros_ws/install/setup.bash
fi

exec "$@"
