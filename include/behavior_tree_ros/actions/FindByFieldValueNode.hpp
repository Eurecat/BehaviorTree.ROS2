#ifndef FIND_BY_FIELD_VALUE_NODE_HPP
#define FIND_BY_FIELD_VALUE_NODE_HPP

#include <algorithm>

#include <behaviortree_cpp/action_node.h>

#include "nlohmann/json.hpp"

namespace BT_ROS
{
class FindByFieldValueNode final : public BT::ActionNodeBase
{
    private:
        using Json = nlohmann::json;

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
            try
            {
                const auto& input  = getParam<nlohmann::json>("input");
                const auto& field  = getParam<std::string>("field");

                nlohmann::json::json_pointer json_pointer(field.value());

                const auto input_type = input.value().front().at(json_pointer).type();
                return find_functions_map_.at(input_type)(input.value(), json_pointer);
            }
            catch(const std::runtime_error&)        { return BT::NodeStatus::FAILURE; }
            catch(const BT::bad_optional_access&)   { return BT::NodeStatus::FAILURE; }
            catch(const nlohmann::json::exception&) { return BT::NodeStatus::FAILURE; }
            catch(const std::out_of_range&)
            {
                throw std::runtime_error { "FindByFieldValueNode: cannot convert value to json entry type" };
            }
        }

        virtual void halt() override {}

    private:
        template <typename T>
        BT::NodeStatus findValue(const Json& _input, const Json::json_pointer& _field)
        {
            const auto& output  = getParam<std::string>("output");
            const auto& value   = getParam<T>("value");
            const auto entry_it = std::find_if(_input.cbegin(), _input.cend(),
                                              [&] (const auto& _json) { return _json.at(_field) == value.value(); });

            if(entry_it == _input.cend()) { return BT::NodeStatus::FAILURE; }

            if(output) { blackboard()->set(output.value(), *entry_it); }

            return BT::NodeStatus::SUCCESS;
        }

    private:
        using FindFunction = std::function<BT::NodeStatus(const Json&, const Json::json_pointer&)>;
        const std::map<Json::value_t, FindFunction> find_functions_map_
        {
            { Json::value_t::boolean,         [this] (const auto& _input, const auto& _field) { return findValue<bool>(_input, _field);        }},
            { Json::value_t::string,          [this] (const auto& _input, const auto& _field) { return findValue<std::string>(_input, _field); }},
            { Json::value_t::number_integer,  [this] (const auto& _input, const auto& _field) { return findValue<int64_t>(_input, _field);     }},
            { Json::value_t::number_unsigned, [this] (const auto& _input, const auto& _field) { return findValue<uint64_t>(_input, _field);    }},
            { Json::value_t::number_float,    [this] (const auto& _input, const auto& _field) { return findValue<double>(_input, _field);      }},
            { Json::value_t::object,          [this] (const auto& _input, const auto& _field) { return findValue<Json>(_input, _field);        }},
            { Json::value_t::array,           [this] (const auto& _input, const auto& _field) { return findValue<Json>(_input, _field);        }},
        };
};
}

#endif
