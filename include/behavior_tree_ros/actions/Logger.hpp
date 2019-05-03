#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <behaviortree_cpp/action_node.h>
#include <ros/ros.h>

namespace BT_ROS
{
template <ros::console::Level LogLevel>
class Logger final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~Logger() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<std::string>("message", "Message to log") };
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);
            const auto& message = getInput<std::string>("message");
            if(!message) { throw BT::RuntimeError { name() + ": " + message.error() }; }

            ROS_LOG(LogLevel, ROSCONSOLE_DEFAULT_NAME, "%s", message.value().c_str());
            return BT::NodeStatus::SUCCESS;
        }
};

using DebugLog = Logger<ros::console::Level::Debug>;
using InfoLog  = Logger<ros::console::Level::Info>;
using WarnLog  = Logger<ros::console::Level::Warn>;
using ErrorLog = Logger<ros::console::Level::Error>;
using FatalLog = Logger<ros::console::Level::Fatal>;
}

#endif
