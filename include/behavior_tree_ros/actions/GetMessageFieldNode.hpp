#ifndef GET_MESSAGE_FIELD_NODE_HPP
#define GET_MESSAGE_FIELD_NODE_HPP

#include <behaviortree_cpp/action_node.h>

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
            //TODO: test setting the output port as void to disable type checking
            return { BT::InputPort<nlohmann::json>("input", "Serialized ROS message"),
                     BT::InputPort<std::string>("field", "Field to fetch"),
                     BT::OutputPort<BT::Any>("output", "Output variable")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& input = getInput<nlohmann::json>("input");
            const auto& field = getInput<std::string>("field");

            //Should input be mandatory too? This could be a problem if messages are yet to be published
            if(!field) { throw BT::RuntimeError { name() + ": " + field.error() }; }
            if(!input) { return BT::NodeStatus::FAILURE; }

            try
            {
                nlohmann::json::json_pointer pointer(field.value().data());
                const auto& json_value = input.value().at(pointer);

                setOutput("output", json2Any(json_value));
                return BT::NodeStatus::SUCCESS;
            }
            catch(const nlohmann::json::exception&) { return BT::NodeStatus::FAILURE; }
        }
};
}

#endif
