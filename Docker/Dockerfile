FROM osrf/ros:jazzy-desktop-full


RUN apt-get update && apt-get upgrade -y
RUN DEBIAN_FRONTEND=noninteractive apt-get install software-properties-common ros-dev-tools python3-colcon-common-extensions -y 
RUN DEBIAN_FRONTEND=noninteractive apt-get install ros-${ROS_DISTRO}-angles ros-${ROS_DISTRO}-tf2 ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-tf2-geometry-msgs
RUN DEBIAN_FRONTEND=noninteractive apt-get install libboost-filesystem1.83-dev
# BT.ROS2 related deps
RUN DEBIAN_FRONTEND=noninteractive apt-get install ros-${ROS_DISTRO}-generate-parameter-library -y
# Boost & yaml-cpp
RUN apt-get install libboost-dev libyaml-dev -y
# Terminator
RUN apt-get install terminator -y
# BT.CPP dependencies
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install libzmq3-dev libsqlite3-dev -y
RUN DEBIAN_FRONTEND=noninteractive apt-get install libncurses5-dev libncursesw5-dev -y
#Groot dependencies
RUN apt-get update && apt-get install qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev -y

COPY entrypoint.sh /entrypoint.sh
# Make sure it is executable
RUN chmod +x /entrypoint.sh

#RUN useradd -m ubuntu
#RUN mkdir -p /home/ubuntu/eut_bt_ros2_ws/src
USER ubuntu
RUN mkdir -p /home/ubuntu/eut_bt_ros2_ws/src/deps
WORKDIR /home/ubuntu/eut_bt_ros2_ws/src


# Environment variables

# Set up workspace
#RUN mkdir -p /eut_bt_ros2_ws/src

# Dont clone "unstable" or frequently chagning repositories in the Dockerfile!



ENTRYPOINT ["/entrypoint.sh"]

CMD ["/bin/bash"]
 
