#ifndef LOGGERS_HPP
#define LOGGERS_HPP

#include <behaviortree_cpp/action_node.h>
#include <ros/ros.h>

//TODO: add more loggers. Maybe using a template class
namespace BT_ROS
{
class InfoLogger final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~InfoLogger() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<std::string>("message", "Message to log") };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& message = getInput<std::string>("message");
            if(!message) { throw BT::RuntimeError { name() + ": " + message.error() }; }

            ROS_INFO("%s", message.value().c_str());
            return BT::NodeStatus::SUCCESS;
        }
};
}

#endif
