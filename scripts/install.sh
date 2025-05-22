#!/bin/bash

# This script is used to build a Docker image and manage dependencies for the 
# BehaviorTree ROS2 project. It performs the following steps:
#
# 1. Sets up the script's working directory and stores the current execution directory.
# 2. Defines paths for the Docker directory and the dependencies directory.
# 3. Accepts an optional build argument (default is 0).
# 4. Builds a Docker image named 'eut_bt_ros2:humble' using the specified Docker directory.
#    - The build argument can be passed to the Docker build process (currently commented out).
#    - The `--no-cache` option is also available but commented out.
# 5. Checks if the dependencies directory exists:
#    - If it does not exist, it creates the directory.
# 6. Executes the `get_deps.sh` script to fetch dependencies.
# 7. Returns to the original execution directory.
#!/bin/bash


SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXEC_DIR="$(pwd)"
cd $SCRIPT_DIR


docker_dir="../Docker/"
docker_depend_dir="../Docker/depend"


# build arguments
arg1=${1:-0}

docker build $docker_dir -t eut_bt_ros2:humble # --build-arg="ARG1=$arg1" #--no-cache


if [ ! -d $docker_depend_dir ] 
then
    mkdir $docker_depend_dir
fi

./get_deps.sh

cd $EXEC_DIR