#ifndef SUBSCRIBER_NODE_HPP
#define SUBSCRIBER_NODE_HPP

//Do not treat reorder as an error even if it's compiled using -Werror
//(this warning comes from ros_type_intronspection itself)
#pragma GCC diagnostic warning "-Wreorder"

#include <topic_tools/shape_shifter.h>
#include <ros_type_introspection/ros_introspection.hpp>

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

            static BT::NodeParameters params { { "topic", "" }, { "queue_size", "1" }, { "key", "" }, {"serialize", "false"} };
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
                serialized_message[field_name] = variant2String(entry.second);
            }

            //std::cout << "Flattened JSON message: " << serialized_message << std::endl;
            //std::cout << "Unflattended JSON message: " << serialized_message.unflatten() << std::endl;

            return serialized_message.unflatten();
        }

        //There has to be a better way
        std::string variant2String(const RosIntrospection::Variant& _variant)
        {
            using namespace RosIntrospection;
            switch(_variant.getTypeID())
            {
                case UINT8:
                    return std::to_string(_variant.extract<uint8_t>());
                    break;
                case UINT16:
                    return std::to_string(_variant.extract<uint16_t>());
                    break;
                case UINT32:
                    return std::to_string(_variant.extract<uint32_t>());
                    break;
                case UINT64:
                    return std::to_string(_variant.extract<uint64_t>());
                    break;
                case BOOL:
                    return std::to_string(_variant.extract<bool>());
                    break;
                case BYTE:
                    return std::to_string(_variant.extract<unsigned char>());
                    break;
                case CHAR:
                    return std::to_string(_variant.extract<char>());
                    break;
                case INT8:
                    return std::to_string(_variant.extract<int8_t>());
                    break;
                case INT16:
                    return std::to_string(_variant.extract<int16_t>());
                    break;
                case INT32:
                    return std::to_string(_variant.extract<int32_t>());
                    break;
                case INT64:
                    return std::to_string(_variant.extract<int64_t>());
                    break;
                case FLOAT32:
                    return std::to_string(_variant.extract<float>());
                case FLOAT64:
                    return std::to_string(_variant.extract<double>());
                    break;
                default:
                    return std::to_string(_variant.convert<double>());
            }
        }

        static const char* msgDefinition() { return ros::message_traits::Definition<MessageType>::value(); };
        static const char* msgDataType()   { return ros::message_traits::DataType<MessageType>::value();   };

        static RosIntrospection::ROSType msgRosType() { return { ros::message_traits::DataType<MessageType>::value() }; };
        static RosIntrospection::Parser& parser() { static RosIntrospection::Parser parser; return parser; };

    private:
        ros::Subscriber subscriber_;

        RosIntrospection::FlatMessage flat_message_;
        std::vector<uint8_t> buffer_;

        bool serialize_;
};
}

#endif
