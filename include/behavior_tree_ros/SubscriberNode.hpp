#ifndef SUBSCRIBER_NODE_HPP
#define SUBSCRIBER_NODE_HPP

#include <topic_tools/shape_shifter.h>

#include "ROSActionNode.hpp"
#include "details/serialization.hpp"

namespace BT_ROS
{
template <class MessageType>
class SubscriberNode final : public ROSActionNode
{
    public:
        using ROSActionNode::ROSActionNode;
        ~SubscriberNode() = default;

        static const NodeParameters& requiredNodeParameters()
        {
            static NodeParameters params { { "topic", "" }, { "queue_size", "1" }, { "key", "" }, { "serialize", "true" } };
            return params;
        }

        virtual BT::NodeStatus tick() override
        {
            return NodeStatus::SUCCESS;
        }

        virtual void onInit() override
        {
            parser().registerMessageDefinition(serialization::msgDataType<MessageType>(),
                                               serialization::msgType<MessageType>(),
                                               serialization::msgDefinition<MessageType>());

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

            parser().deserializeIntoFlatContainer(serialization::msgDataType<MessageType>(),
                                                  absl::Span<uint8_t>(buffer_), &flat_message_, buffer_.size());

            //Serialization is done in to_json() function (serialization.hpp)
            nlohmann::json serialized_json = flat_message_;
            blackboard()->set(getParam<std::string>("key").value() + "_serialized", serialized_json);
        }

        static RosIntrospection::Parser& parser() { static RosIntrospection::Parser parser; return parser; };

    private:
        ros::Subscriber subscriber_;

        RosIntrospection::FlatMessage flat_message_;
        std::vector<uint8_t> buffer_;

        bool serialize_;
};
}

#endif
