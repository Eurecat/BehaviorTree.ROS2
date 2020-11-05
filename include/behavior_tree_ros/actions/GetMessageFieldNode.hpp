#ifndef GET_MESSAGE_FIELD_NODE_HPP
#define GET_MESSAGE_FIELD_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>

#include "behavior_tree_ros/details/conversion_json.hpp"

namespace BT_ROS
{
class GetMessageFieldNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~GetMessageFieldNode() = default;

        static BT::PortsList providedPorts()
        {
            //Seting void as the port type disables type checking
            return { BT::InputPort<nlohmann::json>("input", "Serialized ROS message"),
                     BT::InputPort<std::string>("field", "Field to fetch"),
                     BT::OutputPort<nlohmann::json>("output", "Output variable")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);
            const auto& input = getInput<nlohmann::json>("input");
            const auto& field = getInput<std::string>("field");

            //Should input be mandatory too? This could be a problem if messages are yet to be published
            if(!field) { throw BT::RuntimeError { name() + ": " + field.error() }; }
            if(!input) { return BT::NodeStatus::FAILURE; }

            try
            {
                nlohmann::json::json_pointer pointer(field.value().data());
                const nlohmann::json& json_value = input.value().at(pointer);

                setOutput("output", json_value);
                return BT::NodeStatus::SUCCESS;
            }
            catch(const nlohmann::json::exception&) { return BT::NodeStatus::FAILURE; }
        }
};
}

#endif
