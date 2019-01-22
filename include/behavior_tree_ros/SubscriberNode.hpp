#ifndef SUBSCRIBER_NODE_HPP
#define SUBSCRIBER_NODE_HPP

#include <map>
#include <functional>

//Do not treat reorder as an error even if it's compiled using -Werror
//(this warning comes from ros_type_intronspection itself)
#pragma GCC diagnostic warning "-Wreorder"

#include <ros_type_introspection/ros_introspection.hpp>
#include <topic_tools/shape_shifter.h>

#include "ROSActionNode.hpp"
#include "nlohmann/json.hpp"

namespace BT_ROS
{
template <class MessageType>
class SubscriberNode final : public ROSActionNode
{
    public:
        SubscriberNode(const std::string& _name, const NodeParameters& _params) : ROSActionNode(_name, _params)
        {}
        ~SubscriberNode() = default;

        static const NodeParameters& requiredNodeParameters()
        {
            parser().registerMessageDefinition(msgDataType(), msgRosType(), msgDefinition());

            static NodeParameters params { { "topic", "" }, { "queue_size", "1" }, { "key", "" }, { "serialize", "false" } };
            return params;
        }

        virtual BT::NodeStatus tick() override
        {
            return NodeStatus::SUCCESS;
        }

        virtual void onInit() override
        {
            std::string topic;
            uint32_t queue_size;

            if(!getParam("topic", topic))           { throw std::runtime_error {"Missing topic parameter"}; }
            if(!getParam("queue_size", queue_size)) { throw std::runtime_error {"Missing queue size parameter"}; }
            if(!getParam("serialize", serialize_))  { throw std::runtime_error {"Missing serialize parameter"}; }

            subscriber_ = node_handle_.subscribe(topic, queue_size, &SubscriberNode::callback, this);
        }

        virtual void halt() override {}

    private:
        void callback(const topic_tools::ShapeShifter& _message)
        {
            blackboard()->set(getParam<std::string>("key").value(), *_message.instantiate<MessageType>());
            if(!serialize_) { return; }

            buffer_.resize(_message.size());
            ros::serialization::OStream stream(buffer_.data(), buffer_.size());
            _message.write(stream);

            parser().deserializeIntoFlatContainer(msgDataType(), absl::Span<uint8_t>(buffer_),
                                                  &flat_message_, buffer_.size());

            blackboard()->set(getParam<std::string>("key").value() + "_serialized", toJson(flat_message_));
        }

        //rosInstropection represents messages as trees where the nodes are the message's fields.
        //A path in the tree is a string like: /path/to/vector/field.0/first/entry
        //This can be converted to a valid flat json schema by changing the vectors entries delimiter (point to slash).
        //This flattened json is later unflattened to have a hierarchical structure
        //Type safety is kept in the resulting json
        nlohmann::json toJson(const RosIntrospection::FlatMessage& _flat_message)
        {
            nlohmann::json serialized_message;

            for(const auto& entry : flat_message_.name)
            {
                auto field_name = entry.first.toStdString().substr(msgRosType().baseName().size());
                std::replace(field_name.begin(), field_name.end(), '.', '/');
                serialized_message[field_name] = entry.second;
            }

            for(const auto& entry : flat_message_.value)
            {
                auto field_name = entry.first.toStdString().substr(msgRosType().baseName().size());
                std::replace(field_name.begin(), field_name.end(), '.', '/');

                try
                {
                    variant_to_json_map_.at(entry.second.getTypeID())(field_name, entry.second, serialized_message);
                }
                catch(const std::out_of_range&)
                {
                    throw std::runtime_error { "SubscriberNode: cannot serialize variant field " + field_name
                                               + "of type " + std::string { RosIntrospection::toStr(entry.second.getTypeID()) }};
                }
            }

            return serialized_message.unflatten();
        }

        static const char* msgDefinition() { return ros::message_traits::Definition<MessageType>::value(); };
        static const char* msgDataType()   { return ros::message_traits::DataType<MessageType>::value();   };

        static RosIntrospection::ROSType msgRosType() { return { ros::message_traits::DataType<MessageType>::value() }; };
        static RosIntrospection::Parser& parser()     { static RosIntrospection::Parser parser; return parser; };

    private:
        ros::Subscriber subscriber_;

        RosIntrospection::FlatMessage flat_message_;
        std::vector<uint8_t> buffer_;

        bool serialize_;

        using InsertVariantInJsonFieldFunction = std::function<void(const std::string&, const RosIntrospection::Variant&, nlohmann::json&)>;
        const std::map<RosIntrospection::BuiltinType, InsertVariantInJsonFieldFunction> variant_to_json_map_
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
};
}

#endif
