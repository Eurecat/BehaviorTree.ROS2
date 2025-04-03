#ifndef ROS_LOG_HPP
#define ROS_LOG_HPP

#include "behaviortree_cpp/action_node.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"

namespace BT_ROS
{
template <rclcpp::Logger::Level LogLevel>
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
            const auto& message = getInput<std::string>("message");
            if(!message) { throw BT::RuntimeError { name() + ": " + message.error() }; }

            switch (LogLevel)
            {
                case rclcpp::Logger::Level::Debug:
                    RCLCPP_DEBUG(rclcpp::get_logger("ROS_LOG_DEBUG"), "%s", message.value().c_str());
                    break;
                case rclcpp::Logger::Level::Info:
                    RCLCPP_INFO(rclcpp::get_logger("ROS_LOG_INFO"), "%s", message.value().c_str());
                    break;
                case rclcpp::Logger::Level::Warn:
                    RCLCPP_WARN(rclcpp::get_logger("ROS_LOG_WARN"), "%s", message.value().c_str());
                    break;
                case rclcpp::Logger::Level::Error:
                    RCLCPP_ERROR(rclcpp::get_logger("ROS_LOG_ERROR"), "%s", message.value().c_str());
                    break;
                case rclcpp::Logger::Level::Fatal:
                    RCLCPP_FATAL(rclcpp::get_logger("ROS_LOG_FATAL"), "%s", message.value().c_str());
                    break;
            }
            //ROS_LOG(LogLevel, ROSCONSOLE_DEFAULT_NAME, "%s", message.value().c_str());
            return BT::NodeStatus::SUCCESS;
        }
};

using DebugLog = Logger<rclcpp::Logger::Level::Debug>;
using InfoLog  = Logger<rclcpp::Logger::Level::Info>;
using WarnLog  = Logger<rclcpp::Logger::Level::Warn>;
using ErrorLog = Logger<rclcpp::Logger::Level::Error>;
using FatalLog = Logger<rclcpp::Logger::Level::Fatal>;
}

#endif