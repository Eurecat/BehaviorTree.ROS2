#include <ros/ros.h>

#include "BehaviorTreeServer.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behavior_tree_server_node");

    BT_ROS::BehaviorTreeServer behavior_tree_server ;

    //ros::spin();
    ros::Rate r(10);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}