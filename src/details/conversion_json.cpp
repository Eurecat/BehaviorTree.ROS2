#include <functional>
#include <behaviortree_cpp_v3/utils/demangle_util.h>

#include "behavior_tree_ros/details/conversion_json.hpp"
#include "behavior_tree_ros/utils/UnorderedMap.hpp"
#include "behavior_tree_ros/3rdparty/nlohmann/json.hpp"

namespace BT_ROS
{
    //TODO: improve this a bit, maybe with some template magic?
    //Is there any way to pass a generic functor to all these maps? So far I haven't been able to
    //figure out how, since passing a template functor to the map is a no-no and
    //casting function pointer to other types is UB...
    using Json = nlohmann::json;

    using Json2AnyCastFunctor = std::function<BT::Any(const Json&)>;
    static const Utils::UnorderedMap<Json::value_t, Json2AnyCastFunctor> cast_type_map
    {
        { Json::value_t::boolean,         [] (const auto& _json) { return BT::Any { _json.template get<bool>() }; }},
        { Json::value_t::string,          [] (const auto& _json) { return BT::Any { _json.template get<std::string>() }; }},
        { Json::value_t::number_integer,  [] (const auto& _json) { return BT::Any { _json.template get<int64_t>() }; }},
        { Json::value_t::number_unsigned, [] (const auto& _json) { return BT::Any { _json.template get<uint64_t>() }; }},
        { Json::value_t::number_float,    [] (const auto& _json) { return BT::Any { _json.template get<double>() }; }},
        { Json::value_t::object,          [] (const auto& _json) { return BT::Any { _json }; }},
        { Json::value_t::array,           [] (const auto& _json) { return BT::Any { _json }; }},
    };

    using CompareJsonAnyFunctor = std::function<bool(const Json&, const BT::Any&)>;
    static const Utils::UnorderedMap<Json::value_t, CompareJsonAnyFunctor> compare_map
    {
        { Json::value_t::boolean,         [] (const auto& _json, const auto& _any) { return _json.template get<bool>() == _any.template cast<bool>(); }},
        { Json::value_t::string,          [] (const auto& _json, const auto& _any) { return _json.template get<std::string>() == _any.template cast<std::string>(); }},
        { Json::value_t::number_integer,  [] (const auto& _json, const auto& _any) { return _json.template get<int64_t>() == _any.template cast<int64_t>(); }},
        { Json::value_t::number_unsigned, [] (const auto& _json, const auto& _any) { return _json.template get<uint64_t>() == _any.template cast<uint64_t>(); }},
        { Json::value_t::number_float,    [] (const auto& _json, const auto& _any) { return _json.template get<double>() == _any.template cast<double>(); }},
        { Json::value_t::object,          [] (const auto& _json, const auto& _any) { return _json == _any.template cast<Json>(); }},
        { Json::value_t::array,           [] (const auto& _json, const auto& _any) { return _json == _any.template cast<Json>(); }},
    };

    BT::Any json2Any(const Json& _json)
    {
        try
        {
            return cast_type_map.at(_json.type())(_json);
        }
        catch(const std::out_of_range&)
        {
            throw BT::RuntimeError { std::string { "Cannot convert " } + BT::demangle(typeid(_json))
                                        + " with internal type " + _json.type_name() + " to "
                                        + BT::demangle(typeid(BT::Any)) };
        }
    }

    bool areJsonAndAnyEquals(const Json& _json, const BT::Any& _any)
    {
        try
        {
            return compare_map.at(_json.type())(_json, _any);
        }
        catch(const std::out_of_range&)
        {
            throw BT::RuntimeError { std::string { "Cannot compare " } + BT::demangle(typeid(_json))
                                        + " with internal type " + _json.type_name() + " and "
                                        + BT::demangle(typeid(BT::Any)) + " with internal type "
                                        + BT::demangle(_any.type()) };
        }
        //TODO: check for bad BT::Any conversions
    }
}
