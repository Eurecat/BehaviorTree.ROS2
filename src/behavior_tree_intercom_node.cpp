#include <ros/ros.h>

#include "BehaviorTreeIntercomNode.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behavior_tree_ros_intercom_node");

    BT_ROS::RosHandShake handshake;
    BT_ROS::RosExchangeInfo exchange_info;
    BT_ROS::RosHandShakeData handshakedata;
    ros::spin();

    return 0;
}