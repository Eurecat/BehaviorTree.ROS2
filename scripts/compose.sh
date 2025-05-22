#!/bin/bash

# This script is used to start a Docker container for the "eut_bt_ros2:jazzy" image using docker-compose.
# It performs the following steps:
#
# 1. Determines the directory of the script and stores it in the variable `SCRIPT_DIR`.
# 2. Stores the current working directory in the variable `EXEC_DIR` for later restoration.
# 3. Changes the working directory to the script's directory (`SCRIPT_DIR`).
# 4. Checks if a Docker image with the name "eut_bt_ros2:jazzy" exists:
#    - If the image exists, it navigates to the `../Docker/` directory and starts the container using `docker-compose`.
#    - If the image does not exist, it displays an error message and advises the user to execute the `install.sh` script.
# 5. Restores the original working directory (`EXEC_DIR`).
#
# Usage:
# - Ensure that the Docker image "eut_bt_ros2:jazzy" is built or available before running this script.
# - Run this script from any location; it will automatically adjust paths based on its location.
#
# Prerequisites:
# - Docker and Docker Compose must be installed and properly configured.
# - The `../Docker/docker-compose.yaml` file must exist and define the necessary services.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXEC_DIR="$(pwd)"
cd $SCRIPT_DIR


# Check if a Docker container with the name "eut_bt_ros2:jazzy" exists
if docker images | grep -q "eut_bt_ros2.*jazzy"; then
    echo "Docker image 'eut_bt_ros2:jazzy' found. Starting docker-compose..."
    # Navigate to the ../Docker/ directory and execute docker-compose up
    docker compose -f ../Docker/docker-compose.yaml up 
else
    echo "Error: Docker image 'eut_bt_ros2:jazzy' not found."
    echo "Please execute the install.sh script first."
    exit 1
fi

cd $EXEC_DIR