#ifndef CONVERT_JSON_TO_NODE_HPP
#define CONVERT_JSON_TO_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/utils/safe_any.hpp>
#include "behavior_tree_ros/details/conversion_json.hpp"

namespace BT_ROS
{
template <typename T>
class ConvertJsonToNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~ConvertJsonToNode() = default;

        static BT::PortsList providedPorts()
        {
            //Setting void as the port type disables type checking
            return { BT::InputPort<nlohmann::json>("input", "Serialized ROS message"),
                     BT::OutputPort<T>("output", "Output variable")

            };
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);
            const auto& input = getInput<nlohmann::json>("input");
            
            if(!input) { return BT::NodeStatus::FAILURE; }
   
            const nlohmann::json& json_value = input.value();
            const BT::Any& value = json2Any(json_value);
            
            setOutput("output", value.cast<T>());
            return BT::NodeStatus::SUCCESS;
            
        }
};
}

#endif