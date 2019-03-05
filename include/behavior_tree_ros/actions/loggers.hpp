#ifndef LOGGERS_HPP
#define LOGGERS_HPP

#include <behaviortree_cpp/action_node.h>
#include <ros/ros.h>

//TODO: add more loggers. Maybe using a template class
namespace BT_ROS
{
class InfoLogger final : public BT::ActionNodeBase
{
    public:
        using BT::ActionNodeBase::ActionNodeBase;
        ~InfoLogger() = default;

        static const BT::NodeParameters& requiredNodeParameters()
        {
            static BT::NodeParameters params { { "message", "" } };
            return params;
        }

        virtual BT::NodeStatus tick() override
        {
            try
            {
                const auto& message  = getParam<std::string>("message");
                ROS_INFO("%s", message.value().c_str());
            }
            catch(const BT::bad_optional_access&)   { return BT::NodeStatus::FAILURE; }

            return BT::NodeStatus::SUCCESS;
        }

        virtual void halt() override {}
};
}

#endif
