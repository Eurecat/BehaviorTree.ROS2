#include <std_msgs/String.h>
#include <behavior_tree_demo/TwoNumbers.h>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behavior_tree_ros/behavior_tree_ros.hpp>

BT_REGISTER_NODES(factory)
{
    using namespace BT_ROS;

    factory.registerNodeType<PublisherNode<std_msgs::String>>("DemoPublishString");

    factory.registerNodeType<SubscriberNode<behavior_tree_demo::TwoNumbers>>("DemoMonitorTwoNumbers");
}
