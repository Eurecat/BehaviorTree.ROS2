#ifndef BEHAVIOR_TREE_ROS_DESERIALIZATION
#define BEHAVIOR_TREE_ROS_DESERIALIZATION

#include <functional>
#include <behaviortree_cpp/action_node.h>

#include "behavior_tree_ros/3rdparty/nlohmann/json.hpp"
#include "behavior_tree_ros/utils/UnorderedMap.hpp"

namespace BT_ROS
{
namespace deserialization
{
    using Json = nlohmann::json;
    using DeserializeFieldFunction = std::function<void(BT::ActionNodeBase&, const std::string&, const Json&)>;

    template <typename FieldType>
    inline void deserializeField(BT::ActionNodeBase& _node, const std::string& _port, const Json& _field)
    {
        _node.setOutput(_port, _field.get<FieldType>());
    }

    static const Utils::UnorderedMap<Json::value_t, DeserializeFieldFunction> deserialize_field_map
    {
        { Json::value_t::boolean,         [] (auto& _node, const auto& _port, const auto& _field) { deserializeField<bool>(_node, _port, _field);        }},
        { Json::value_t::string,          [] (auto& _node, const auto& _port, const auto& _field) { deserializeField<std::string>(_node, _port, _field); }},
        { Json::value_t::number_integer,  [] (auto& _node, const auto& _port, const auto& _field) { deserializeField<int64_t>(_node, _port, _field);     }},
        { Json::value_t::number_unsigned, [] (auto& _node, const auto& _port, const auto& _field) { deserializeField<uint64_t>(_node, _port, _field);    }},
        { Json::value_t::number_float,    [] (auto& _node, const auto& _port, const auto& _field) { deserializeField<double>(_node, _port, _field);      }},
        { Json::value_t::object,          [] (auto& _node, const auto& _port, const auto& _field) { deserializeField<Json>(_node, _port, _field);        }},
        { Json::value_t::array,           [] (auto& _node, const auto& _port, const auto& _field) { deserializeField<Json>(_node, _port, _field);        }},
    };

    inline void deserializeField(BT::ActionNodeBase& _node, const std::string& _port, const Json& _field)
    {
        deserialize_field_map.at(_field.type())(_node, _port, _field);
    }

}
}

#endif
