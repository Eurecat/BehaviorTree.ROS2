#ifndef FIND_BY_FIELD_VALUE_NODE_HPP
#define FIND_BY_FIELD_VALUE_NODE_HPP

#include <algorithm>
#include <behaviortree_cpp/action_node.h>

#include "behavior_tree_ros/utils/definitions.hpp"
#include "behavior_tree_ros/details/conversion_json.hpp"

namespace BT_ROS
{
class FindByFieldValueNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~FindByFieldValueNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<nlohmann::json>("input", "Serialized ROS message"),
                     BT::InputPort<BT::StringView>("field", "Field to fetch"),
                     BT::InputPort<BT::Any>("value", "Value to search for"),
                     BT::OutputPort<BT::Any>("output", "Output variable")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& input = getInput<nlohmann::json>("input");
            const auto& field = getInput<BT::StringView>("field");
            const auto& value = getInput<BT::Any>("field");

            if(!input) { return BT::NodeStatus::FAILURE; }
            if(!field) { return BT::NodeStatus::FAILURE; }
            if(!value) { return BT::NodeStatus::FAILURE; }

            try
            {
                nlohmann::json::json_pointer json_pointer(field.value().data());

                //TODO:finish this
                /*
                const auto entry_it = std::find_if(input.value().cbegin(), input.value().cend(),
                                     [&] (const auto& _json) { return _json.at(json_pointer) == value.value().cast<_json.at(json_pointer)>; });

                if(entry_it == input.value().cend()) { return BT::NodeStatus::FAILURE; }

                setOutput("output", json2Any(*entry_it));
                */
                return BT::NodeStatus::SUCCESS;
            }
            catch(const nlohmann::json::exception&) { return BT::NodeStatus::FAILURE; }
        }
};
}

#endif
