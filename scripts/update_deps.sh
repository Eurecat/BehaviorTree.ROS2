#!/bin/bash


# This script is used to manage dependencies for a project by ensuring that specific Git repositories
# are either cloned or updated to their latest versions. It operates within a Docker dependency directory.

# Steps performed by the script:
# 1. Determine the directory of the script (`SCRIPT_DIR`) and the directory where the script was executed (`EXEC_DIR`).
# 2. Navigate to the script directory to ensure relative paths are resolved correctly.
# 3. Define the path to the Docker dependency directory (`docker_depend_dir`).

# For each dependency (e.g., `behavior_tree_eut_plugins` and `groot`):
# - Check if the repository exists locally by verifying the presence of a `.git` directory.
# - If the repository exists:
#   - Navigate to the repository directory.
#   - Pull the latest changes from the remote repository.
#   - Return to the previous working directory.
# - If the repository does not exist:
#   - Clone the repository from the specified GitLab URL using the `humble` branch.
#   - Place the cloned repository in the appropriate subdirectory under `docker_depend_dir`.

# Finally, return to the original execution directory (`EXEC_DIR`).

# Note:
# - The script assumes SSH access to the GitLab server (`git@gitlab.local.eurecat.org`).
# - Ensure that the user running the script has the necessary permissions and SSH keys configured.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXEC_DIR="$(pwd)"

cd $SCRIPT_DIR
docker_depend_dir="../deps"

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
