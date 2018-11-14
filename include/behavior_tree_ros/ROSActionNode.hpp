#ifndef ROS_ACTION_NODE_HPP
#define ROS_ACTION_NODE_HPP

#include <string>
#include <ros/ros.h>

#include "behavior_tree_core/action_node.h"

namespace BT_ROS
{
class ROSActionNode : public BT::ActionNodeBase
{
    public:
        ROSActionNode(const std::string& _name, const BT::NodeParameters& _params) : ActionNodeBase(_name, _params) {}
        virtual ~ROSActionNode() = default;

        template <typename MessageType>
        friend MessageType buildMessage(const ROSActionNode& _ros_node);

    protected:
        ros::NodeHandle node_handle_;
};

}

#endif
