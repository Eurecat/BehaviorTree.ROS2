#include <ros/ros.h>

#include "BehaviorTreeServer.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behavior_tree_server_node");

    BT_ROS::BehaviorTreeServer behavior_tree_server ;

   // ros::spin();
    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}