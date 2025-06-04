#!/bin/bash
ROS_DISTRO=jazzy


set -e
mkdir -p deps

cd deps

vcs import . < /deps.repos
vcs pull .

ROS_SETUP="source /opt/ros/${ROS_DISTRO}/setup.bash"
if ! grep -Fxq "$ROS_SETUP" ~/.bashrc; then
  echo "$ROS_SETUP" >> ~/.bashrc
fi
#
OVERLAY_SETUP="source /home/ubuntu/eut_bt_ros2_ws/install/setup.bash"
if ! grep -Fxq "$OVERLAY_SETUP" ~/.bashrc; then
  echo "$OVERLAY_SETUP" >> ~/.bashrc
fi
#
AUTOCOMPLETE_SETUP="source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
if ! grep -Fxq "$AUTOCOMPLETE_SETUP" ~/.bashrc; then
  echo "$AUTOCOMPLETE_SETUP" >> ~/.bashrc
fi

WORKING_DIR="cd /home/ubuntu/eut_bt_ros2_ws"
if ! grep -Fxq "$WORKING_DIR" ~/.bashrc; then
  echo "$WORKING_DIR" >> ~/.bashrc
fi

cd ../../

source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install
source ~/.bashrc

echo "SYSTEM READY"
exec "$@"
