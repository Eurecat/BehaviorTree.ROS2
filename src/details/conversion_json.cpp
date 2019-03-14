#include <functional>

#include "behavior_tree_ros/details/conversion_json.hpp"
#include "behavior_tree_ros/utils/definitions.hpp"
#include "nlohmann/json.hpp"

namespace BT_ROS
{
    using Json = nlohmann::json;
    using CastTypeFunctor = std::function<BT::Any(const Json&)>;
    static const Utils::UnorderedMap<Json::value_t, CastTypeFunctor> cast_type_map
    {
        { Json::value_t::boolean,         [] (const auto& _json) { return BT::Any { _json.template get<bool>() }; }},
        { Json::value_t::string,          [] (const auto& _json) { return BT::Any { _json.template get<std::string>() }; }},
        { Json::value_t::number_integer,  [] (const auto& _json) { return BT::Any { _json.template get<int64_t>() }; }},
        { Json::value_t::number_unsigned, [] (const auto& _json) { return BT::Any { _json.template get<uint64_t>() }; }},
        { Json::value_t::number_float,    [] (const auto& _json) { return BT::Any { _json.template get<double>() }; }},
        { Json::value_t::object,          [] (const auto& _json) { return BT::Any { _json }; }},
        { Json::value_t::array,           [] (const auto& _json) { return BT::Any { _json }; }},
    };

    BT::Any json2Any(const nlohmann::json& _json)
    {
        try
        {
            return cast_type_map.at(_json.type())(_json);
        }
        catch(const std::out_of_range&)
        {
            throw BT::RuntimeError { "Cannot cast" };
        }
    }
}
