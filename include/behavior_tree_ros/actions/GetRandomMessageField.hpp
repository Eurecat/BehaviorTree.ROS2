#ifndef GET_RANDOM_MESSAGE_FIELD_NODE_HPP
#define GET_RANDOM_MESSAGE_FIELD_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>

#include "behavior_tree_ros/utils/random.hpp"
#include "behavior_tree_ros/details/conversion_json.hpp"

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
                     BT::OutputPort<nlohmann::json>("output", "Output variable")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);
            const auto& input  = getInput<nlohmann::json>("input");
            const auto& field  = getInput<std::string>("field");

            //Should input be mandatory too? This may cause issues when messages are yet to be published
            if(!field) { throw BT::RuntimeError { name() + ": " + field.error() }; }
            if(!input) { return BT::NodeStatus::FAILURE; }

            try
            {
                nlohmann::json::json_pointer pointer(field.value().data());
                const auto& json_entry = input.value().at(pointer);
                const auto random_it   = Utils::getRandomIterator(json_entry.cbegin(), json_entry.cend());

                setOutput("output", *random_it);

                return BT::NodeStatus::SUCCESS;
            }
            catch(const nlohmann::json::exception&) { return BT::NodeStatus::FAILURE; }
        }
};
}

#endif
