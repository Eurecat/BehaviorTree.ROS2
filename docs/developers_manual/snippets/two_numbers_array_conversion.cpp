#include <behavior_tree_demo/TwoNumbersArray.h>

#include <behavior_tree_ros/behavior_tree_ros.hpp>

namespace BT_ROS
{
    template <>
    NodeParameters requiredMessageParameters<behavior_tree_demo::TwoNumbersArray>() { return { { "first",  "0.0"}, { "second", "0.0"}, { "third",  "0.0"}, { "fourth", "0.0"} }; }

    template <>
    behavior_tree_demo::TwoNumbersArray buildMessage(const ROSActionNode& _ros_node)
    {
        behavior_tree_demo::TwoNumbersArray array_message {};
        array_message.array.resize(2);

        array_message.array[0].numbers[0].data = _ros_node.getParam<float>("first").value();
        array_message.array[0].numbers[1].data = _ros_node.getParam<float>("second").value();
        array_message.array[1].numbers[0].data = _ros_node.getParam<float>("third").value();
        array_message.array[1].numbers[1].data = _ros_node.getParam<float>("fourth").value();

        array_message.array[0].header.stamp = ros::Time::now();
        array_message.array[1].header.stamp = ros::Time::now();

        return array_message;
    }
}
