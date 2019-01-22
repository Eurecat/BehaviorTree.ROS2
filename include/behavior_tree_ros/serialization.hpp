#ifndef BEHAVIOR_TREE_ROS_SERIALIZATION
#define BEHAVIOR_TREE_ROS_SERIALIZATION

//Do not treat reorder as an error even if it's compiled using -Werror
//(this warning comes from ros_type_intronspection itself)
#pragma GCC diagnostic warning "-Wreorder"

#include <ros_type_introspection/ros_introspection.hpp>

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

    //TODO: do not create multiple ROSMessage
    template <class MessageType>
    inline RosIntrospection::ROSMessage msgInfo()
    {
        return { msgDefinition<MessageType>() };
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
}
}

#endif
