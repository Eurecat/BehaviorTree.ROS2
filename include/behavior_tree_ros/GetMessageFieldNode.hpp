#ifndef GET_MESSAGE_FIELD_NODE_HPP
#define GET_MESSAGE_FIELD_NODE_HPP

#include <map>
#include <functional>

#include <behaviortree_cpp/action_node.h>

#include "nlohmann/json.hpp"

namespace BT_ROS
{
class GetMessageFieldNode final : public BT::ActionNodeBase
{
    public:
        GetMessageFieldNode(const std::string& _name, const BT::NodeParameters& _params) : ActionNodeBase(_name, _params)
        {}
        ~GetMessageFieldNode() = default;

        static const BT::NodeParameters& requiredNodeParameters()
        {
            static BT::NodeParameters params { { "input", "" }, { "field", "" }, { "output", "" } };
            return params;
        }

        virtual BT::NodeStatus tick() override
        {
            try
            {
                const auto& input  = getParam<nlohmann::json>("input");
                const auto& field  = getParam<std::string>("field");
                const auto& output = getParam<std::string>("output");

                nlohmann::json::json_pointer pointer(field.value());
                const auto& json_value = input.value().at(pointer);
                insert_functions_map_.at(json_value.type())(output.value(), json_value);

                return BT::NodeStatus::SUCCESS;
            }
            catch(const std::runtime_error&)        { return BT::NodeStatus::FAILURE; }
            catch(const BT::bad_optional_access&)   { return BT::NodeStatus::FAILURE; }
            catch(const nlohmann::json::exception&) { return BT::NodeStatus::FAILURE; }
            catch(const std::out_of_range&)
            { 
                throw std::runtime_error { "GetMessageFieldNode: cannot convert json field" };
            }
        }

        virtual void halt() override {}

    private:
        using Json = nlohmann::json;
        using InsertFunction = std::function<void(const std::string&, const Json&)>;
        const std::map<Json::value_t, InsertFunction> insert_functions_map_
        {
            { Json::value_t::boolean,         [this] (const auto& _key, const auto& _json) { blackboard()->set(_key, _json.template get<bool>()); }},
            { Json::value_t::string,          [this] (const auto& _key, const auto& _json) { blackboard()->set(_key, _json.template get<std::string>()); }},
            { Json::value_t::number_integer,  [this] (const auto& _key, const auto& _json) { blackboard()->set(_key, _json.template get<int64_t>()); }},
            { Json::value_t::number_unsigned, [this] (const auto& _key, const auto& _json) { blackboard()->set(_key, _json.template get<uint64_t>()); }},
            { Json::value_t::number_float,    [this] (const auto& _key, const auto& _json) { blackboard()->set(_key, _json.template get<double>()); }},
            { Json::value_t::object,          [this] (const auto& _key, const auto& _json) { blackboard()->set(_key, _json); }},
            { Json::value_t::array,           [this] (const auto& _key, const auto& _json) { blackboard()->set(_key, _json); }}, //Should be converted to vector?
        };
};
}

#endif
