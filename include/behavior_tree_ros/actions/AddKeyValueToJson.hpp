#ifndef ADD_KEY_VALUE_TO_NODE
#define ADD_KEY_VALUE_TO_NODE

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/utils/safe_any.hpp>
#include "behavior_tree_ros/details/conversion_json.hpp"

namespace BT_ROS
{
class AddKeyValueToJson final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~AddKeyValueToJson() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<std::string>("input_key", "Input key name"),
                     BT::InputPort<std::string>("input_value", "Input value"),
                     BT::InputPort<nlohmann::json>("input_json", "Input json to copy to"),
                     BT::OutputPort<nlohmann::json>("output", "Output json with new value") };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& input_key = getInput<std::string>("input_key");
            const auto& input_value = getInput<std::string>("input_value");
            if(!input_key || !input_value) { throw BT::RuntimeError { name() + ": missing one or more required fields" }; }
            
            // If there's no input_json an error is thrown when getting the variable
            // catch the error and do nothing to use an empty json as input_json
            nlohmann::json json_store {};
            try
            {
                const auto& input_json = getInput<nlohmann::json>("input_json");
                json_store = input_json.value();
            }
            catch(const nlohmann::json::exception&) {}
            
            try{
                // Fill Json according to its type
                if (json_store.is_array())
                {
                    nlohmann::json object;
                    object[input_key.value()] = input_value.value();
                    json_store.push_back(object);
                }
                else
                    json_store[input_key.value()] = input_value.value();

                setOutput("output", json_store);
                return BT::NodeStatus::SUCCESS;
            }catch(const nlohmann::json::exception&) {
                std::cout << "Error when updating JSON"; 
                return BT::NodeStatus::FAILURE; }
        }
};

}

#endif
