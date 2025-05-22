#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXEC_DIR="$(pwd)"

cd $SCRIPT_DIR
./get_deps.sh
cd $EXEC_DIR


