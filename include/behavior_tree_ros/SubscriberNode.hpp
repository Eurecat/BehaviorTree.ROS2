#ifndef SUBSCRIBER_NODE_HPP
#define SUBSCRIBER_NODE_HPP

//Do not treat reorder as an error even if it's compiled using -Werror
//(the warning comes from ros_type_introspection)
#pragma GCC diagnostic warning "-Wreorder"

#include <topic_tools/shape_shifter.h>
#include <ros_type_introspection/ros_introspection.hpp>

#include "ROSActionNode.hpp"

namespace BT_ROS
{
template <class MessageType>
class SubscriberNode final : public ROSActionNode
{

    public:
        SubscriberNode(const std::string& _name, const BT::NodeParameters& _params) : ROSActionNode(_name, _params)
        {}
        ~SubscriberNode() = default;

        static const BT::NodeParameters& requiredNodeParameters()
        {
            static BT::NodeParameters params { {"topic", ""} };

            parser().registerMessageDefinition(msgDataType(), msgRosType(), msgDefinition());
            const auto& definition = parser().getMessageInfo(msgDataType());

            for(const auto& message : definition->type_list)
            {
                for(const auto& field : message.fields())
                {
                    if(field.type().isBuiltin())
                    {
                        params.emplace(field.name() + "_key", field.name());
                    }
                }
            }

            return params;
        }

        virtual BT::NodeStatus tick() override
        {
            if(subscriber_) { return BT::NodeStatus::SUCCESS; }

            setStatus(BT::NodeStatus::RUNNING);

            try
            { 
                subscribeToTopic();
            }
            catch(const std::runtime_error&)      { setStatus(BT::NodeStatus::FAILURE); }
            catch(const BT::bad_optional_access&) { setStatus(BT::NodeStatus::FAILURE); }

            return status();
        }

        virtual void halt() override {}

    private:
        void subscribeToTopic()
        {
            std::string topic;
            if(!getParam("topic", topic)) { throw std::runtime_error {"Missing topic name"}; }

            subscriber_ = node_handle_.subscribe(topic, 1, &SubscriberNode::callback, this);
        }

        void callback(const topic_tools::ShapeShifter& _msg)
        {
            buffer_.resize(_msg.size());
            ros::serialization::OStream stream(buffer_.data(), buffer_.size());
            _msg.write(stream);

            parser().deserializeIntoFlatContainer(msgDataType(), absl::Span<uint8_t>(buffer_),
                                                  &flat_message_, buffer_.size());

            for(const auto& entry : flat_message_.name)
            {
                //TODO: test string fields
                blackboard()->set(entry.first.toStdString(), entry.second);
            }

            //TODO: handle vectors
            for(const auto& entry : flat_message_.value)
            {
                const auto& field_name = entry.first.node_ptr->value();
                blackboard()->set(getParam<std::string>(field_name + "_key").value(),
                                  entry.second.convert<double>());
            }
        }

        static const char* msgDefinition()            { return ros::message_traits::Definition<MessageType>::value(); };
        static const char* msgDataType()              { return ros::message_traits::DataType<MessageType>::value();   };
        static RosIntrospection::ROSType msgRosType() { return { ros::message_traits::DataType<MessageType>::value() }; };

        static RosIntrospection::Parser& parser() { static RosIntrospection::Parser parser; return parser; }

    private:
        BT::optional<ros::Subscriber> subscriber_;
        std::vector<uint8_t> buffer_;

        RosIntrospection::FlatMessage flat_message_;
};

}

#endif
