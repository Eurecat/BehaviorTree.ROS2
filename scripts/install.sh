#!/bin/bash


SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXEC_DIR="$(pwd)"
cd $SCRIPT_DIR


docker_dir="../Docker/"
docker_depend_dir="../Docker/depend"

docker build $docker_dir -t eut_bt_ros2:humble $1 

cd $EXEC_DIR
