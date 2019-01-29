#include <std_msgs/String.h>
#include <behavior_tree_demo/TwoNumbers.h>
#include <behavior_tree_demo/TwoNumbersArray.h>

#include <behaviortree_cpp/bt_factory.h>
#include <behavior_tree_ros/behavior_tree_ros.hpp>

BT_REGISTER_NODES(factory)
{
    using namespace BT_ROS;

    factory.registerNodeType<PublisherNode<std_msgs::String>>("DemoPublishString");
    factory.registerNodeType<PublisherNode<behavior_tree_demo::TwoNumbersArray, false>>("DemoPublishTwoNumbersArray");

    factory.registerNodeType<SubscriberNode<behavior_tree_demo::TwoNumbers>>("DemoMonitorTwoNumbers");
}
