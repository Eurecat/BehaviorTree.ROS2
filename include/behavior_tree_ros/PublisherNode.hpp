#ifndef PUBLISHER_NODE_HPP
#define PUBLISHER_NODE_HPP

#pragma GCC diagnostic warning "-Wreorder"

#include <topic_tools/shape_shifter.h>
#include <ros_type_introspection/ros_introspection.hpp>

#include <behaviortree_cpp/basic_types.h>

#include "ROSActionNode.hpp"
#include "conversion_types.hpp"

namespace BT_ROS
{
template <class MessageType>
class PublisherNode final : public ROSActionNode
{
    public:
        PublisherNode(const std::string& _name, const NodeParameters& _params) : ROSActionNode(_name, _params)
        {
            shape_shifter_.morph(ros::message_traits::MD5Sum<MessageType>::value(),
                                 ros::message_traits::DataType<MessageType>::value(),
                                 ros::message_traits::Definition<MessageType>::value(), "" );
        }
        ~PublisherNode() = default;

        static const NodeParameters& requiredNodeParameters()
        {
            static NodeParameters params { { "topic", "" }, { "queue_size", "1" }, { "latch", "false" } };

            for(const auto& field : MsgInfo().fields())
            {
                if(field.isConstant()) { continue; }
                params[field.name()] = "";
            }
            
            //const auto& message_parameters = requiredMessageParameters<MessageType>();
            //params.insert(message_parameters.cbegin(), message_parameters.cend());

            return params;
        }

        virtual NodeStatus tick() override
        {
            try
            {
                //const auto& message = buildMessage<MessageType>(*this);
                //publisher_.publish(message);

                for(const auto& field : MsgInfo().fields())
                {
                    serialize_field_map_.at(field.type().typeID())(field);
                }

                ros::serialization::OStream stream(serialization_buffer_.data(), serialization_buffer_.size());
                shape_shifter_.read(stream);
                publisher_.publish(shape_shifter_);

                serialization_buffer_.clear();
            }
            catch(const std::runtime_error&)      { return NodeStatus::FAILURE; }
            catch(const BT::bad_optional_access&) { return NodeStatus::FAILURE; }
            catch(const std::out_of_range&)
            {
                throw std::runtime_error { "PublisherNode: unrecognized field type in message " +  std::string { msgDataType() }
                                            + ". Non-builtin types automatic serialization is not supported."
                                            + " Implement specializations for buildMessage<> and requiredMessageParameters<> functions instead." };
            }

            return NodeStatus::SUCCESS;
        }

        virtual void onInit() override
        {
            std::string topic;
            uint32_t queue_size;
            bool latch;

            if(!getParam("topic", topic))           { throw std::runtime_error { "Missing topic parameter" }; }
            if(!getParam("queue_size", queue_size)) { throw std::runtime_error { "Missing queue size parameter" }; }
            if(!getParam("latch", latch))           { throw std::runtime_error { "Missing latch parameter" }; }

            publisher_ = node_handle_.advertise<MessageType>(topic, queue_size, latch);
        }

        virtual void halt() override {}

        static const char* msgDefinition() { return ros::message_traits::Definition<MessageType>::value(); };
        static const char* msgDataType()   { return ros::message_traits::DataType<MessageType>::value();   };

        static RosIntrospection::ROSMessage& MsgInfo() { static RosIntrospection::ROSMessage message_info(msgDefinition()); return message_info; };

    private:
        template <typename T>
        void serializeField(const RosIntrospection::ROSField& _field)
        {
            if(_field.isConstant()) { return; }

            const auto& field_value = getParam<T>(_field.name());

            const auto current_length = serialization_buffer_.size();
            const auto field_length   = ros::serialization::serializationLength(field_value.value());
            serialization_buffer_.resize(current_length + field_length);

            ros::serialization::OStream stream(serialization_buffer_.data() + current_length, field_length);
            ros::serialization::serialize(stream, field_value.value());
        }

    private:
        ros::Publisher publisher_;
        topic_tools::ShapeShifter shape_shifter_;

        std::vector<uint8_t> serialization_buffer_;

        using SerializeFieldFunction = std::function<void(const RosIntrospection::ROSField&)>;
        const std::map<RosIntrospection::BuiltinType, SerializeFieldFunction> serialize_field_map_
        {
            //TODO: Fix serialization error with char
            { RosIntrospection::UINT8,    [this] (const auto& _field) { serializeField<uint8_t>(_field);     }},
            { RosIntrospection::UINT16,   [this] (const auto& _field) { serializeField<uint16_t>(_field);    }},
            { RosIntrospection::UINT32,   [this] (const auto& _field) { serializeField<uint32_t>(_field);    }},
            { RosIntrospection::UINT64,   [this] (const auto& _field) { serializeField<uint64_t>(_field);    }},
            { RosIntrospection::BOOL,     [this] (const auto& _field) { serializeField<bool>(_field);        }},
            { RosIntrospection::BYTE,     [this] (const auto& _field) { serializeField<uint8_t>(_field);     }},
            //{ RosIntrospection::CHAR,     [this] (const auto& _field) { serializeField<char>(_field);     }},
            { RosIntrospection::INT8,     [this] (const auto& _field) { serializeField<int8_t>(_field);      }},
            { RosIntrospection::INT16,    [this] (const auto& _field) { serializeField<int8_t>(_field);      }},
            { RosIntrospection::INT32,    [this] (const auto& _field) { serializeField<int8_t>(_field);      }},
            { RosIntrospection::INT64,    [this] (const auto& _field) { serializeField<int8_t>(_field);      }},
            { RosIntrospection::FLOAT32,  [this] (const auto& _field) { serializeField<float>(_field);       }},
            { RosIntrospection::FLOAT64,  [this] (const auto& _field) { serializeField<double>(_field);      }},
            { RosIntrospection::STRING,   [this] (const auto& _field) { serializeField<std::string>(_field); }},
            { RosIntrospection::TIME,     [] (const auto& _field) {}}, //Don't do anything
            { RosIntrospection::DURATION, [] (const auto& _field) {}}, //Don't do anything
        };
};
}

#endif
