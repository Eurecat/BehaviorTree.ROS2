#!/bin/bash
ROS_DISTRO=jazzy


set -e
mkdir -p deps

cd deps

#if [ -d "behavior_tree_eut_plugins/.git" ]; then
#    echo "Repository behavior_tree_eut_plugins exists. Pulling latest changes..."
#    cd behavior_tree_eut_plugins
#    git pull
#    cd ..
#else
#    rm -rf behavior_tree_eut_plugins
#    echo "Repository behavior_tree_eut_plugins does not exist. Cloning..."
#    git clone  git@gitlab.local.eurecat.org:robotics-automation/behavior_tree_eut_plugins.git --branch jazzy
#fi
#
#if [ -d "rosx_introspection/.git" ]; then
#    echo "Repository rosx_introspection exists. Pulling latest changes..."
#    cd "rosx_introspection"
#    git pull
#    cd ..
#else
#    rm -rf rosx_introspection
#    echo "Repository rosx_introspection does not exist. Cloning..."
#    git clone https://github.com/eurecat/rosx_introspection.git
#fi
#
#if [ -d "BehaviorTree.CPP/.git" ]; then
#    echo "Repository BehaviorTree.CPP exists. Pulling latest changes..."
#    cd "BehaviorTree.CPP"
#    git pull
#    cd ..
#else
#    rm -rf BehaviorTree.CPP
#    echo "Repository BehaviorTree.CPP does not exist. Cloning..."
#    git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git --branch 4.6.2 --single-branch
#fi
#
#if [ -d "groot/.git" ]; then
#    echo "Repository groot exists. Pulling latest changes..."
#    cd groot
#    git pull
#    cd ..
#else
#    rm -rf groot
#    echo "Repository groot does not exist. Cloning..."
#    git clone -b humble git@gitlab.local.eurecat.org:robotics-automation/groot.git 
#fi
##chown -R $HOST_UID:$HOST_GID groot
#
#

vcs import . < ../deps.repos

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
