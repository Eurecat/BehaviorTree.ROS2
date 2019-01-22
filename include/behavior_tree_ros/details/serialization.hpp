#ifndef BEHAVIOR_TREE_ROS_SERIALIZATION
#define BEHAVIOR_TREE_ROS_SERIALIZATION

#include <map>
#include <functional>

//Do not treat reorder as an error even if it's compiled using -Werror
//(this warning comes from ros_type_intronspection itself)
#pragma GCC diagnostic warning "-Wreorder"
#include <ros_type_introspection/ros_introspection.hpp>

#include "nlohmann/json.hpp"

namespace BT_ROS
{
namespace serialization
{
    template <class MessageType>
    inline const char* msgDefinition()
    {
        return ros::message_traits::Definition<MessageType>::value();
    };

    template <class MessageType>
    inline const char* msgDataType()
    {
        return ros::message_traits::DataType<MessageType>::value();
    };

    template <class MessageType>
    inline const char* msgMD5Sum()
    {
        return ros::message_traits::MD5Sum<MessageType>::value();
    };

    template <class MessageType>
    inline RosIntrospection::ROSMessage msgInfo()
    {
        return { msgDefinition<MessageType>() };
    };

    template <class MessageType>
    inline RosIntrospection::ROSType msgType()
    {
        return { msgDataType<MessageType>() };
    };

    template <typename MessageType>
    inline void serializeField(const ROSActionNode& _node, const RosIntrospection::ROSField& _field, std::vector<uint8_t>& _buffer)
    {
        if(_field.isConstant()) { return; }

        const auto& field_value   = _node.getParam<MessageType>(_field.name());
        const auto current_length = _buffer.size();
        const auto field_length   = ros::serialization::serializationLength(field_value.value());
        _buffer.resize(current_length + field_length);

        ros::serialization::OStream stream(_buffer.data() + current_length, field_length);
        ros::serialization::serialize(stream, field_value.value());
    }

    using SerializeFieldFunction = std::function<void(const ROSActionNode&, const RosIntrospection::ROSField&, std::vector<uint8_t>&)>;
    static const std::map<RosIntrospection::BuiltinType, SerializeFieldFunction> serialize_field_map
    {
        { RosIntrospection::UINT8,    [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<uint8_t>(_node, _field, _buffer);     }},
        { RosIntrospection::UINT16,   [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<uint16_t>(_node, _field, _buffer);    }},
        { RosIntrospection::UINT32,   [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<uint32_t>(_node, _field, _buffer);    }},
        { RosIntrospection::UINT64,   [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<uint64_t>(_node, _field, _buffer);    }},
        { RosIntrospection::BOOL,     [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<bool>(_node, _field, _buffer);        }},
        { RosIntrospection::BYTE,     [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<int8_t>(_node, _field, _buffer);      }},
        { RosIntrospection::CHAR,     [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<uint8_t>(_node, _field, _buffer);     }},
        { RosIntrospection::INT8,     [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<int8_t>(_node, _field, _buffer);      }},
        { RosIntrospection::INT16,    [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<int16_t>(_node, _field, _buffer);      }},
        { RosIntrospection::INT32,    [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<int32_t>(_node, _field, _buffer);      }},
        { RosIntrospection::INT64,    [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<int64_t>(_node, _field, _buffer);      }},
        { RosIntrospection::FLOAT32,  [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<float>(_node, _field, _buffer);       }},
        { RosIntrospection::FLOAT64,  [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<double>(_node, _field, _buffer);      }},
        { RosIntrospection::STRING,   [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<std::string>(_node, _field, _buffer); }},
        { RosIntrospection::TIME,     [] (const auto& _node, const auto& _field, auto& _buffer) {}}, //Don't do anything
        { RosIntrospection::DURATION, [] (const auto& _node, const auto& _field, auto& _buffer) {}}, //Don't do anything
    };

    inline void serializeField(const ROSActionNode& _node, const RosIntrospection::ROSField& _field, std::vector<uint8_t>& _buffer)
    {
        serialize_field_map.at(_field.type().typeID())(_node, _field, _buffer);
    }

    using InsertVariantInJsonFieldFunction = std::function<void(const std::string&, const RosIntrospection::Variant&, nlohmann::json&)>;
    static const std::map<RosIntrospection::BuiltinType, InsertVariantInJsonFieldFunction> variant_to_json_map
    {
        { RosIntrospection::UINT8,    [] (const auto& _field_name, const auto& _variant, auto& _json) { _json.emplace(_field_name, _variant.template extract<uint8_t>()); }},
        { RosIntrospection::UINT16,   [] (const auto& _field_name, const auto& _variant, auto& _json) { _json.emplace(_field_name, _variant.template extract<uint16_t>()); }},
        { RosIntrospection::UINT32,   [] (const auto& _field_name, const auto& _variant, auto& _json) { _json.emplace(_field_name, _variant.template extract<uint32_t>()); }},
        { RosIntrospection::UINT64,   [] (const auto& _field_name, const auto& _variant, auto& _json) { _json.emplace(_field_name, _variant.template extract<uint64_t>()); }},
        { RosIntrospection::BOOL,     [] (const auto& _field_name, const auto& _variant, auto& _json) { _json.emplace(_field_name, _variant.template extract<bool>()); }},
        { RosIntrospection::BYTE,     [] (const auto& _field_name, const auto& _variant, auto& _json) { _json.emplace(_field_name, _variant.template extract<int8_t>()); }},
        { RosIntrospection::CHAR,     [] (const auto& _field_name, const auto& _variant, auto& _json) { _json.emplace(_field_name, _variant.template extract<uint8_t>()); }},
        { RosIntrospection::INT8,     [] (const auto& _field_name, const auto& _variant, auto& _json) { _json.emplace(_field_name, _variant.template extract<int8_t>()); }},
        { RosIntrospection::INT16,    [] (const auto& _field_name, const auto& _variant, auto& _json) { _json.emplace(_field_name, _variant.template extract<int16_t>()); }},
        { RosIntrospection::INT32,    [] (const auto& _field_name, const auto& _variant, auto& _json) { _json.emplace(_field_name, _variant.template extract<int32_t>()); }},
        { RosIntrospection::INT64,    [] (const auto& _field_name, const auto& _variant, auto& _json) { _json.emplace(_field_name, _variant.template extract<int64_t>()); }},
        { RosIntrospection::FLOAT32,  [] (const auto& _field_name, const auto& _variant, auto& _json) { _json.emplace(_field_name, _variant.template extract<float>()); }},
        { RosIntrospection::FLOAT64,  [] (const auto& _field_name, const auto& _variant, auto& _json) { _json.emplace(_field_name, _variant.template extract<double>()); }},
        { RosIntrospection::TIME,     [] (const auto& _field_name, const auto& _variant, auto& _json) {}}, //Don't do anything
        { RosIntrospection::DURATION, [] (const auto& _field_name, const auto& _variant, auto& _json) {}}, //Don't do anything
    };

    inline void deserializeField(const std::string& _field_name, const RosIntrospection::Variant& _value, nlohmann::json& _json)
    {
        variant_to_json_map.at(_value.getTypeID())(_field_name, _value, _json);
    }
}
}

//rosInstropection represents messages as trees where the nodes are the message's fields.
//A path in the tree is a string like: /path/to/vector/field.0/first/entry
//This can be converted to a valid flat json schema by changing the vectors entries delimiter (point to slash).
//This flattened json can be later unflattened to have a hierarchical structure
//Type safety is ensured
namespace nlohmann
{
    template <>
    struct adl_serializer<RosIntrospection::FlatMessage>
    {
        static void to_json(json& _json, const RosIntrospection::FlatMessage& _flat_message)
        {
            const auto& base_name = _flat_message.tree->croot()->value();

            for(const auto& entry : _flat_message.name)
            {
                auto field_name = entry.first.toStdString().substr(base_name.size());
                std::replace(field_name.begin(), field_name.end(), '.', '/');
                _json[field_name] = entry.second;
            }

            for(const auto& entry : _flat_message.value)
            {
                auto field_name = entry.first.toStdString().substr(base_name.size());
                std::replace(field_name.begin(), field_name.end(), '.', '/');

                try
                {
                    BT_ROS::serialization::deserializeField(field_name, entry.second, _json);
                }
                catch(const std::out_of_range&)
                {
                    throw std::runtime_error { "Cannot serialize variant field " + field_name + "of type "
                                                + std::string { RosIntrospection::toStr(entry.second.getTypeID()) }};
                }
            }

            _json = _json.unflatten();
        }
    };
}

#endif
