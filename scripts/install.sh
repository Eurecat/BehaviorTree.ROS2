#!/bin/bash


SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXEC_DIR="$(pwd)"
cd $SCRIPT_DIR

echo "UID=$(id -u)" > ../Docker/.env 
echo "GID=$(id -g)" >> ../Docker/.env
echo "NAME=user" >> ../Docker/.env

docker_dir="../Docker/"
docker_depend_dir="../Docker/depend"

docker build $docker_dir -t eut_bt_ros2:jazzy $1 

cd $EXEC_DIR
