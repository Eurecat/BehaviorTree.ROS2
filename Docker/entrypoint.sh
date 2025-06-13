#!/bin/bash
ROS_DISTRO=jazzy

set -e

# Set default UID and GID if not provided
USER_ID=${UID:-1000}
GROUP_ID=${GID:-1000}
USERNAME=${UNAME:-ubuntu}
GROUPNAME=${UNAME:-ubuntu}
user_removed=false


# Check if the user already exists
EXISTING_USERNAME=$(getent passwd "$USER_ID" | cut -d: -f1)

# If the user exists and the username is different, remove the existing user (we need to free the UID)
if [ -n "$EXISTING_USERNAME" ] && [ "$EXISTING_USERNAME" != "$USERNAME" ]; then
  echo "Removing user $EXISTING_USERNAME"
  deluser --remove-home $EXISTING_USERNAME
  user_removed=true
fi

# The development folder has shared in the docker-compose file as volume
# It is not a user folder, so we need to add the initial files like .bashrc and .profile
if [ "$user_removed" = true ]; then
  cp -r /etc/skel/. /home/$USERNAME/ 2>/dev/null
fi

# Change the ownership of the home directory to the specified user and group
chown $USER_ID:$GROUP_ID /home/$USERNAME/

### Create group if it doesn't exist
if ! getent group $GROUP_ID >/dev/null; then
    groupadd -g $GROUP_ID $USERNAME
fi

# Now, we can create the user with the specified UID and GID
if ! id -u $USER_ID >/dev/null 2>&1; then
  echo "Creating user $USERNAME with UID $USER_ID and GID $GROUP_ID"
    useradd -m -u $USER_ID -g $GROUP_ID -s /bin/bash $USERNAME
fi

# If not present, request to load the ROS environment in the .bashrc file
ROS_SETUP="source /opt/ros/${ROS_DISTRO}/setup.bash"
if ! grep -Fxq "$ROS_SETUP" /home/$USERNAME/.bashrc; then
  echo "$ROS_SETUP" >> ~/.bashrc
fi

# If not present, request to load the workspace environment in the .bashrc file
OVERLAY_SETUP="source /home/${USERNAME}/eut_bt_ros2_ws/install/setup.bash"
if ! grep -Fxq "$OVERLAY_SETUP" /home/$USERNAME/.bashrc; then
  echo "$OVERLAY_SETUP" >> /home/$USERNAME/.bashrc
fi

# If not present, request to load the colcon_argcomplete environment in the .bashrc file
AUTOCOMPLETE_SETUP="source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
if ! grep -Fxq "$AUTOCOMPLETE_SETUP" /home/$USERNAME/.bashrc; then
  echo "$AUTOCOMPLETE_SETUP" >> /home/$USERNAME/.bashrc
fi


#WORKING_DIR="cd /home/${USERNAME}/eut_bt_ros2_ws"
#if ! grep -Fxq "$WORKING_DIR" /home/$USERNAME/.bashrc; then
#  echo "$WORKING_DIR" >> /home/$USERNAME/.bashrc
#fi
  

# Set the ownership of the home directory and recurisvely the volumes set in the docker-compose file
chown $USERNAME:$GROUPNAME /home/$USERNAME/* -R


# Switch to the new user and run the main command
exec gosu "$USERNAME" bash -c "
  echo 'Running as $USERNAME...';

  # Load the environment variables
  source /home/$USERNAME/.bashrc;

  # Open the workspace and join the dependencies folder after created
  cd /home/$USERNAME/eut_bt_ros2_ws/src/;
  mkdir deps;
  cd deps;

  # Download the dependencies from the repository
  vcs import . < /deps.repos;
  vcs pull .;

  # Compile the workspace
  cd ../../;
  source /opt/ros/${ROS_DISTRO}/setup.bash;
  colcon build --symlink-install;
  
  exec \"$@\";
"