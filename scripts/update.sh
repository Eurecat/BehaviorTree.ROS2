#!/bin/bash


# This script is used to update dependencies for a project.
# It performs the following steps:
# 1. Determines the directory where the script is located (SCRIPT_DIR).
# 2. Stores the current working directory (EXEC_DIR).
# 3. Changes the working directory to the script's directory (SCRIPT_DIR).
# 4. Executes the `get_deps.sh` script to fetch or update dependencies.
# 5. Returns to the original working directory (EXEC_DIR).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXEC_DIR="$(pwd)"

cd $SCRIPT_DIR
./get_deps.sh
cd $EXEC_DIR


