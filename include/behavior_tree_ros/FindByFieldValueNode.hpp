#ifndef FIND_BY_FIELD_VALUE_NODE_HPP
#define FIND_BY_FIELD_VALUE_NODE_HPP

#include <algorithm>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/action_node.h>

#include "nlohmann/json.hpp"

namespace BT_ROS
{
class FindByFieldValueNode final : public BT::ActionNodeBase
{
    public:
        FindByFieldValueNode(const std::string& _name, const BT::NodeParameters& _params) : ActionNodeBase(_name, _params)
        {}
        ~FindByFieldValueNode() = default;

        static const BT::NodeParameters& requiredNodeParameters()
        {
            static BT::NodeParameters params { { "input", "" }, { "field", "" },
                                               { "value", "" }, { "output", "" } };
            return params;
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);

            try
            {
                const auto& input  = getParam<nlohmann::json>("input");
                const auto& field  = getParam<std::string>("field");
                const auto& value  = getParam<std::string>("value");
                const auto& output = getParam<std::string>("output");

                nlohmann::json::json_pointer json_pointer(field.value());

                const auto entry_it = std::find_if(input.value().cbegin(), input.value().cend(),
                                                  [&] (const auto& _json) { return _json.at(json_pointer) == value.value(); });

                if(entry_it == input.value().cend()) { return BT::NodeStatus::FAILURE; }

                blackboard()->set(output.value(), *entry_it);
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
