#ifndef SUBSCRIBER_NODE_HPP
#define SUBSCRIBER_NODE_HPP

#include "ROSActionNode.hpp"
#include "details/serialization.hpp"

namespace BT_ROS
{
template <class MessageType>
class SubscriberNode final : public ROSActionNode
{
    public:
        SubscriberNode(const std::string& _name, const BT::NodeConfiguration& _config) : ROSActionNode(_name, _config)
        {
            parser().registerMessageDefinition(serialization::msgDataType<MessageType>(),
                                               serialization::msgType<MessageType>(),
                                               serialization::msgDefinition<MessageType>());

            const auto& topic      = getInput<std::string>("topic");
            const auto& queue_size = getInput<uint32_t>("queue_size");
            const auto& serialize  = getInput<bool>("serialize");

            if(!topic)      {  throw BT::RuntimeError { std::string{ "SubscriberNode<" } + serialization::msgDataType<MessageType>() + ">: " + topic.error() }; }
            if(!queue_size) {  throw BT::RuntimeError { std::string{ "SubscriberNode<" } + serialization::msgDataType<MessageType>() + ">: " + queue_size.error() }; }
            if(!serialize)  {  throw BT::RuntimeError { std::string{ "SubscriberNode<" } + serialization::msgDataType<MessageType>() + ">: " + serialize.error() }; }

            serialize_  = serialize.value();
            subscriber_ = node_handle_.subscribe(topic.value(), queue_size.value(), &SubscriberNode::callback, this);
        }

        ~SubscriberNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<nlohmann::json>("topic", "Topic to subscribe"),
                     BT::InputPort<uint32_t>("queue_size", 1, "Subscriber callback queue size"),
                     BT::InputPort<bool>("serialize", false, "Serialize ROS message?"),
                     BT::OutputPort<MessageType>("output", "Received message"),
                     BT::OutputPort<nlohmann::json>("serialized_output", "Serialized ROS message output")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            return BT::NodeStatus::SUCCESS;
        }

        virtual void halt() override {};

    private:
        void callback(const MessageType& _message)
        {
            setOutput("output", _message);

            if(!serialize_) { return; }

            //Note: is it possible to receive the serialized data directly?
            buffer_.resize(ros::serialization::serializationLength(_message));
            ros::serialization::OStream stream(buffer_.data(), buffer_.size());
            ros::serialization::serialize(stream, _message);

            parser().deserializeIntoFlatContainer(serialization::msgDataType<MessageType>(),
                                                  absl::Span<uint8_t>(buffer_), &flat_message_, buffer_.size());

            //Serialization is done in to_json() function (serialization.hpp)
            nlohmann::json serialized_json = flat_message_;
            setOutput("serialized_output", serialized_json);
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
