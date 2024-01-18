#ifndef ADD_ARRAY_TO_JSON
#define ADD_ARRAY_TO_JSON

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/utils/safe_any.hpp>
#include "behavior_tree_ros/details/conversion_json.hpp"

namespace BT_ROS
{
class AddArrayToJson final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~AddArrayToJson() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<bool>("override", true, "1 to override the actual array; 0 to append value"),
                     BT::InputPort<std::string>("input_key", "", "Key name of the array"),
                     BT::InputPort<std::string>("input_value", "New value or value to append to the array"),
                     BT::InputPort("input_json", "Input Json array"),
                     BT::OutputPort<nlohmann::json>("output_json", "Output Json array")
            };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& override = getInput<bool>("override");
            const auto& input_key = getInput<std::string>("input_key");
            const auto& input_value = getInput<std::string>("input_value");
            const auto& input_json = getInput<std::string>("input_json");

            if (!override) { throw BT::RuntimeError { name() + ": " + override.error() }; }
            if (!input_value) { throw BT::RuntimeError { name() + ": " + input_value.error() }; }

            try
            {
                // Check if input value has json format in case it's an array or object already
                nlohmann::json input_value_json;
                bool value_is_json = false;
                try {
                    input_value_json = nlohmann::json::parse(input_value.value());
                    value_is_json = true;
                } catch(const nlohmann::json::exception& ex) {}

                // Check if input Json is empty
                // If you try to parse an empty variable an exception is thrown
                nlohmann::json output_json;
                if (!input_json.value().empty()) { output_json = nlohmann::json::parse(input_json.value()); }

                if(output_json.is_object()) // Json is an object
                {
                    // Clear array if we have to override it
                    if (override.value()) { output_json[input_key.value()].clear(); }

                    if(value_is_json) { output_json[input_key.value()].push_back(input_value_json); }
                    else { output_json[input_key.value()].push_back(input_value.value()); }
                }
                else if(output_json.is_array()) // Json is an array
                {
                    if (input_key.value_or("") != "") // Add array with key
                    {
                        bool filled = false;
                        // Check all the elements of the array for an object with the same key
                        for(auto& elem : output_json)
                            if(elem.is_object())
                                if(elem.find(input_key.value()) != elem.end())
                                {
                                    const nlohmann::json::iterator& key_elem = elem.find(input_key.value());

                                    if (override.value()) { key_elem.value().clear(); }

                                    if(value_is_json) { key_elem.value().push_back(input_value_json); }
                                    else { key_elem.value().push_back(input_value.value()); }

                                    filled = true;
                                    break;
                                }
                        // If there was no object, create a new object with the array
                        if(!filled)
                        {
                            nlohmann::json object;
                            object[input_key.value()].push_back(input_value.value());
                            output_json.push_back(object);
                        }
                    }
                    else // Add array without key
                    {
                        if (override.value()) { output_json.clear(); }

                        if(value_is_json) { output_json.push_back(input_value_json); }
                        else { output_json.push_back(input_value.value()); }
                    }
                }
                else // Empty Json
                {
                    if (input_key.value_or("") != "")
                    {
                        if(value_is_json) { output_json[input_key.value()].push_back(input_value_json); }
                        else { output_json[input_key.value()].push_back(input_value.value()); }
                    }
                    else
                    {
                        if(value_is_json) { output_json.push_back(input_value_json); }
                        else { output_json.push_back(input_value.value()); }
                    }
                }

                setOutput("output_json", output_json);
                return  BT::NodeStatus::SUCCESS;
            }
            catch (const nlohmann::json::exception& ex) { return BT::NodeStatus::FAILURE; }
        }
};
}

#endif