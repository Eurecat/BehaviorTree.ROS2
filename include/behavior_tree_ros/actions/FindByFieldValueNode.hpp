#ifndef FIND_BY_FIELD_VALUE_NODE_HPP
#define FIND_BY_FIELD_VALUE_NODE_HPP

#include <algorithm>
#include <behaviortree_cpp/action_node.h>

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
                     BT::InputPort<std::string>("field", "Field to fetch"),
                     BT::InputPort<nlohmann::json>("value", "Value to search for"),
                     BT::OutputPort<nlohmann::json>("output", "Output variable")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);

            const auto& input = getInput<nlohmann::json>("input");
            const auto& field = getInput<std::string>("field");
            //TODO: do not use string for this
            const auto& value = getInput<nlohmann::json>("value");

            //Should input be mandatory too? This may cause issues when messages are yet to be published
            if(!field) { throw BT::RuntimeError { name() + ": " + field.error() }; }
            if(!value) { throw BT::RuntimeError { name() + ": " + value.error() }; }
            if(!input) { throw BT::RuntimeError { name() + ": " + input.error() }; }

            try
            {
                nlohmann::json::json_pointer json_pointer(field.value().data());

                //TODO: fix this conversion to string
                const auto entry_it = std::find_if(input.value().cbegin(), input.value().cend(),
                                     [&] (const auto& _json)
                                     { return _json.at(json_pointer).dump() == value.value().get<std::string>(); });

                if(entry_it == input.value().cend()) { return BT::NodeStatus::FAILURE; }

                setOutput("output", json2Any(*entry_it));
                return BT::NodeStatus::SUCCESS;
            }
            catch(const nlohmann::json::exception&) { return BT::NodeStatus::FAILURE; }
        }
};
}

#endif
