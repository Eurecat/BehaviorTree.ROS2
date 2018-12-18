#ifndef GET_RANDOM_MESSAGE_FIELD_NODE_HPP
#define GET_RANDOM_MESSAGE_FIELD_NODE_HPP

#include <behaviortree_cpp/action_node.h>

#include "nlohmann/json.hpp"
#include "utils/random.hpp"

namespace BT_ROS
{
class GetRandomMessageFieldNode final : public BT::ActionNodeBase
{
    public:
        GetRandomMessageFieldNode(const std::string& _name, const BT::NodeParameters& _params) : ActionNodeBase(_name, _params)
        {}
        ~GetRandomMessageFieldNode() = default;

        static const BT::NodeParameters& requiredNodeParameters()
        {
            static BT::NodeParameters params { { "input", "" }, { "field", "" }, { "output", "" } };
            return params;
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);

            try
            {
                const auto& input  = getParam<nlohmann::json>("input");
                const auto& field  = getParam<std::string>("field");
                const auto& output = getParam<std::string>("output");

                nlohmann::json::json_pointer pointer(field.value());
                const auto& json_entry = input.value().at(pointer);

                blackboard()->set(output.value(), *Utils::getRandomIterator(json_entry.cbegin(), json_entry.cend()));

                return BT::NodeStatus::SUCCESS;
            }
            catch(const std::runtime_error&)        { return BT::NodeStatus::FAILURE; }
            catch(const BT::bad_optional_access&)   { return BT::NodeStatus::FAILURE; }
            catch(const nlohmann::json::exception&) { return BT::NodeStatus::FAILURE; }
        }

        virtual void halt() override {}
};
}

#endif
