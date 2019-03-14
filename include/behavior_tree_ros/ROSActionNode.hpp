#ifndef ROS_ACTION_NODE_HPP
#define ROS_ACTION_NODE_HPP

#include <ros/ros.h>
#include <behaviortree_cpp/action_node.h>

#include "details/conversion_types.hpp"

namespace BT_ROS
{
template <class MessageType>
BT::PortsList requiredMessagePorts();

class ROSActionNode: public BT::ActionNodeBase
{
    public:
        using BT::ActionNodeBase::ActionNodeBase;
        virtual ~ROSActionNode() = default;

        template <class MessageType>
        friend MessageType buildMessage(const ROSActionNode& _ros_node);

    protected:
        ros::NodeHandle node_handle_;
};
}

#endif
