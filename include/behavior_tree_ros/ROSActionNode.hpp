#ifndef ROS_ACTION_NODE_HPP
#define ROS_ACTION_NODE_HPP

#include <string>
#include <ros/ros.h>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

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
        ROSActionNode(const std::string& _name, const NodeParameters& _params) : ActionNodeBase(_name, _params) {}
        virtual ~ROSActionNode() = default;

        template <typename MessageType>
        friend MessageType buildMessage(const ROSActionNode& _ros_node);

    protected:
        ros::NodeHandle node_handle_;
};

}

#endif
