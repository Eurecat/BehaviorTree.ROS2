#ifndef ROS_ACTION_NODE_HPP
#define ROS_ACTION_NODE_HPP

#include <ros/ros.h>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include "details/conversion_types.hpp"

namespace BT_ROS
{
using NodeParameters    = BT::NodeParameters;
using MessageParameters = BT::NodeParameters;
using NodeStatus        = BT::NodeStatus;

template <class MessageType>
NodeParameters requiredMessageParameters();

class ROSActionNode : public BT::ActionNodeBase
{
    public:
        using BT::ActionNodeBase::ActionNodeBase;
        virtual ~ROSActionNode() = default;

        template <typename MessageType>
        friend MessageType buildMessage(const ROSActionNode& _ros_node);

    protected:
        ros::NodeHandle node_handle_;
};
}

#endif
