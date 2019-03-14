#ifndef GET_RANDOM_MESSAGE_FIELD_NODE_HPP
#define GET_RANDOM_MESSAGE_FIELD_NODE_HPP

#include <behaviortree_cpp/action_node.h>

#include "nlohmann/json.hpp"
#include "behavior_tree_ros/utils/random.hpp"

namespace BT_ROS
{
class GetRandomMessageFieldNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~GetRandomMessageFieldNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<nlohmann::json>("input", "Serialized ROS message"),
                     BT::InputPort<std::string>("field", "Field to fetch"),
                     BT::OutputPort<std::string>("output", "Output variable")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            try
            {
                const auto& input  = getInput<nlohmann::json>("input");
                const auto& field  = getInput<std::string>("field");
                const auto& output = getInput<std::string>("output");

                nlohmann::json::json_pointer pointer(field.value());
                const auto& json_entry = input.value().at(pointer);

                setOutput(output.value(), *Utils::getRandomIterator(json_entry.cbegin(), json_entry.cend()));

                return BT::NodeStatus::SUCCESS;
            }
            catch(const std::runtime_error&)        { return BT::NodeStatus::FAILURE; }
            //catch(const BT::bad_optional_access&)   { return BT::NodeStatus::FAILURE; }
            catch(const nlohmann::json::exception&) { return BT::NodeStatus::FAILURE; }
        }
};
}

#endif
