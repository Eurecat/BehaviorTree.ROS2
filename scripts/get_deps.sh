#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXEC_DIR="$(pwd)"

cd $SCRIPT_DIR
docker_depend_dir="../Docker/depend"

if [ -d "$docker_depend_dir/behavior_tree_eut_plugins/.git" ]; then
    echo "Repository behavior_tree_eut_plugins exists. Pulling latest changes..."
    cd "$docker_depend_dir/behavior_tree_eut_plugins"
    git pull
    cd $OLDPWD
else
    echo "Repository behavior_tree_eut_plugins does not exist. Cloning..."
    git clone -b humble git@gitlab.local.eurecat.org:robotics-automation/behavior_tree_eut_plugins.git "$docker_depend_dir/behavior_tree_eut_plugins"
fi

if [ -d "$docker_depend_dir/groot/.git" ]; then
    echo "Repository groot exists. Pulling latest changes..."
    cd "$docker_depend_dir/groot"
    git pull
    cd $OLDPWD
else
    echo "Repository groot does not exist. Cloning..."
    git clone -b humble git@gitlab.local.eurecat.org:robotics-automation/groot.git "$docker_depend_dir/groot"
fi

cd $EXEC_DIR