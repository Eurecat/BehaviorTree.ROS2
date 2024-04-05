#ifndef BEHAVIOR_TREE_ROS_SERIALIZATION
#define BEHAVIOR_TREE_ROS_SERIALIZATION

#include <functional>
#include <behaviortree_cpp_v3/action_node.h>

//Do not treat reorder as an error even if it's compiled using -Werror
//(this warning comes from ros_type_intronspection itself)
#pragma GCC diagnostic warning "-Wreorder"
#pragma GCC diagnostic warning "-Wsign-compare"
#include <ros_type_introspection/ros_introspection.hpp>

#include "behavior_tree_ros/3rdparty/nlohmann/json.hpp"
#include "behavior_tree_ros/utils/UnorderedMap.hpp"

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

    //Helper function to check if a message is empty (useful to detect empty services responses)
    template <class MessageType>
    inline bool isMsgEmpty()
    {
        return strcmp(msgDefinition<MessageType>(), "\n") == 0;
    };

    template <typename FieldType>
    inline void serializeField(const FieldType& _value, std::vector<uint8_t>& _buffer)
    {
        const auto current_length = _buffer.size();
        const auto field_length   = ros::serialization::serializationLength(_value);
        _buffer.resize(current_length + field_length);

        ros::serialization::OStream stream(_buffer.data() + current_length, field_length);
        ros::serialization::serialize(stream, _value);
    }

    template <typename FieldType>
    inline void serializeField(const BT::ActionNodeBase& _node, const std::string& _port, std::vector<uint8_t>& _buffer)
    {
        const auto& field_value = _node.getInput<FieldType>(_port);
        if(!field_value) throw BT::RuntimeError(BT::StrCat("serializeField with port ", _port, ". Error: ", field_value.error()));
        serializeField<FieldType>(field_value.value(), _buffer);
    }

    using SerializeFieldFunction = std::function<void(const BT::ActionNodeBase&, const std::string&, std::vector<uint8_t>&)>;
    static const Utils::UnorderedMap<RosIntrospection::BuiltinType, SerializeFieldFunction> serialize_field_map
    {
        { RosIntrospection::UINT8,    [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<uint8_t>(_node, _field, _buffer);     }},
        { RosIntrospection::UINT16,   [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<uint16_t>(_node, _field, _buffer);    }},
        { RosIntrospection::UINT32,   [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<uint32_t>(_node, _field, _buffer);    }},
        { RosIntrospection::UINT64,   [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<uint64_t>(_node, _field, _buffer);    }},
        { RosIntrospection::BOOL,     [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<bool>(_node, _field, _buffer);        }},
        { RosIntrospection::BYTE,     [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<int8_t>(_node, _field, _buffer);      }},
        { RosIntrospection::CHAR,     [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<uint8_t>(_node, _field, _buffer);     }},
        { RosIntrospection::INT8,     [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<int8_t>(_node, _field, _buffer);      }},
        { RosIntrospection::INT16,    [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<int16_t>(_node, _field, _buffer);     }},
        { RosIntrospection::INT32,    [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<int32_t>(_node, _field, _buffer);     }},
        { RosIntrospection::INT64,    [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<int64_t>(_node, _field, _buffer);     }},
        { RosIntrospection::FLOAT32,  [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<float>(_node, _field, _buffer);       }},
        { RosIntrospection::FLOAT64,  [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<double>(_node, _field, _buffer);      }},
        { RosIntrospection::STRING,   [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField<std::string>(_node, _field, _buffer); }},
        { RosIntrospection::TIME,     [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField(ros::Time::now(), _buffer);           }}, // Use default value
        { RosIntrospection::DURATION, [] (const auto& _node, const auto& _field, auto& _buffer) { serializeField(ros::Duration(), _buffer);            }}, // Use default value
    };

    inline void serializeField(const BT::ActionNodeBase& _node, const std::string& _port,
                               const RosIntrospection::BuiltinType& _type, std::vector<uint8_t>& _buffer)
    {
        serialize_field_map.at(_type)(_node, _port, _buffer);
    }

    using PortData     = std::pair<std::string, BT::PortInfo>;
    using PortFunction = std::function<PortData(const BT::PortDirection, const std::string&, const std::string&)>;
    static const Utils::UnorderedMap<RosIntrospection::BuiltinType, PortFunction> generate_port_map
    {
        { RosIntrospection::UINT8,    [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<uint8_t>(_direction, _name, _description);  }},
        { RosIntrospection::UINT16,   [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<uint16_t>(_direction, _name, _description); }},
        { RosIntrospection::UINT32,   [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<uint32_t>(_direction, _name, _description); }},
        { RosIntrospection::UINT64,   [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<uint64_t>(_direction, _name, _description); }},
        { RosIntrospection::BOOL,     [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<bool>(_direction, _name, _description);     }},
        { RosIntrospection::BYTE,     [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<int8_t>(_direction, _name, _description);   }},
        { RosIntrospection::CHAR,     [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<uint8_t>(_direction, _name, _description);  }},
        { RosIntrospection::INT8,     [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<int8_t>(_direction, _name, _description);   }},
        { RosIntrospection::INT16,    [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<int16_t>(_direction, _name, _description);  }},
        { RosIntrospection::INT32,    [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<int32_t>(_direction, _name, _description);  }},
        { RosIntrospection::INT64,    [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<int64_t>(_direction, _name, _description);  }},
        { RosIntrospection::FLOAT32,  [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<float>(_direction, _name, _description);    }},
        { RosIntrospection::FLOAT64,  [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<double>(_direction, _name, _description);   }},
        { RosIntrospection::STRING,   [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<std::string>(_direction, _name, _description); }},
    };

    inline PortData getTypedPort(const RosIntrospection::ROSField& _field,
                                 const BT::PortDirection _port_direction,
                                 const std::string& _port_name,
                                 const std::string& _port_description)
    {
        return generate_port_map.at(_field.type().typeID())(_port_direction, _port_name, _port_description);
    }

    using InsertVariantInJsonFieldFunction = std::function<void(const std::string&, const RosIntrospection::Variant&, nlohmann::json&)>;
    static const Utils::UnorderedMap<RosIntrospection::BuiltinType, InsertVariantInJsonFieldFunction> variant_to_json_map
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
    struct FlatMessageWithIgnoredFields
    {
        FlatMessageWithIgnoredFields(const RosIntrospection::FlatMessage& base_flat_message, const int ignore_fields_size = 0)
            : 
            flat_msg_(base_flat_message),
            ignore_fields_(std::vector<std::string>(ignore_fields_size))
        {} 

        FlatMessageWithIgnoredFields(const RosIntrospection::FlatMessage& base_flat_message, const std::vector<std::string>& ignore_fields)
            : FlatMessageWithIgnoredFields(flat_msg_, ignore_fields.size())
        {
            for(const std::string& field : ignore_fields)
                ignore_fields_.push_back(field);
        } 

        const RosIntrospection::FlatMessage& flat_msg_;
        std::vector<std::string> ignore_fields_;
    };

    template <>
    struct adl_serializer<FlatMessageWithIgnoredFields>
    {
        static void to_json(json& _json, const FlatMessageWithIgnoredFields& _flat_message_ignore_fields)
        {
            // /result/avatars/0/data/1
            // /result/avatars
            auto fieldShallBeIgnore = [&_flat_message_ignore_fields](const std::string& field_name) -> bool {
                for(const auto& prefix_to_ignore : _flat_message_ignore_fields.ignore_fields_)
                {
                    if(field_name.find(prefix_to_ignore.c_str()) != std::string::npos) return true;
                }
                return false;
            };

            const auto& base_name = _flat_message_ignore_fields.flat_msg_.tree->croot()->value();

            for(const auto& entry : _flat_message_ignore_fields.flat_msg_.name)
            {
                auto field_name = entry.first.toStdString().substr(base_name.size());
                std::replace(field_name.begin(), field_name.end(), '.', '/');
                _json[field_name] = entry.second;
            }

            for(const auto& entry : _flat_message_ignore_fields.flat_msg_.value)
            {
                auto field_name = entry.first.toStdString().substr(base_name.size());
                std::replace(field_name.begin(), field_name.end(), '.', '/');

                try
                {
                    if(fieldShallBeIgnore(field_name)) continue; //field to be ignored

                    BT_ROS::serialization::deserializeField(field_name, entry.second, _json);
                }
                catch(const std::out_of_range&)
                {
                    throw BT::RuntimeError { "Cannot serialize variant field " + field_name + " of type "
                                                + std::string { RosIntrospection::toStr(entry.second.getTypeID()) }};
                }
            }

            //ROS empty messages/services responses are not considered objects
            if(!_json.is_object()) { return; }
            _json = _json.unflatten();
        };
    };
}

#endif
