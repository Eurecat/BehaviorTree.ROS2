#ifndef SPLIT_STRING_TO_JSON_ARRAY
#define SPLIT_STRING_TO_JSON_ARRAY

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/utils/safe_any.hpp>
#include "behavior_tree_ros/details/conversion_json.hpp"

namespace BT_ROS
{
class SplitStringToJsonArray final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~SplitStringToJsonArray() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<std::string>("delimiter", ";", "delimiter to split the sequence"),
                     BT::InputPort<std::string>("sequence", "Sequence to split"),
                     BT::OutputPort<uint64_t>("json_size", "Number of elements from the split"),
                     BT::OutputPort<nlohmann::json>("output_json", "Json array with the sequence splitted")

            };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& delimiter = getInput<std::string>("delimiter");
            const auto& sequence = getInput<std::string>("sequence");

            if (!delimiter) { throw BT::RuntimeError { name() + ": " + delimiter.error() }; }
            if (!sequence) { throw BT::RuntimeError { name() + ": " + sequence.error() }; }

            std::string string_sequence = sequence.value();
            if (string_sequence.find(delimiter.value()) == std::string::npos) { return BT::NodeStatus::FAILURE; }

            try
            {
                nlohmann::json output_json;
                size_t pos = 0;
                std::string sub_string;
                while ((pos = string_sequence.find(delimiter.value())) != std::string::npos)
                {
                    sub_string = string_sequence.substr(0, pos);
                    output_json.push_back(sub_string);
                    string_sequence.erase(0, pos + delimiter.value().length());
                }

                // Append the last splitted part
                if (!string_sequence.empty()) { output_json.push_back(string_sequence); }

                setOutput("json_size", output_json.size());
                setOutput("output_json", output_json);
                return BT::NodeStatus::SUCCESS;
            }
            catch(const nlohmann::json::exception& ex) { return BT::NodeStatus::FAILURE; }
        }

}; // class SplitStringToJsonArray

} // namespace BT_ROS

#endif // SPLIT_STRING_TO_JSON_ARRAY