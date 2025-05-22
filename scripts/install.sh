#!/bin/bash

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