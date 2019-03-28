#ifndef SUBSCRIBER_NODE_HPP
#define SUBSCRIBER_NODE_HPP

#include <behaviortree_cpp/action_node.h>

#include "behavior_tree_ros/policies/serialization_policies.hpp"
#include "behavior_tree_ros/details/conversion_types.hpp"

namespace BT_ROS
{
template <class MessageType, template <class> class SerializationPolicy>
class SubscriberNode final : public BT::ActionNodeBase, public SerializationPolicy<MessageType>
{
    public:
        SubscriberNode(const std::string& _name, const BT::NodeConfiguration& _config) : ActionNodeBase(_name, _config)
        {
            const auto& topic      = getInput<std::string>("topic");
            const auto& queue_size = getInput<uint32_t>("queue_size");

            if(!topic)      { throw BT::RuntimeError { name() + ": " + topic.error() };      }
            if(!queue_size) { throw BT::RuntimeError { name() + ": " + queue_size.error() }; }

            subscriber_ = node_handle_.subscribe(topic.value(), queue_size.value(), &SubscriberNode::callback, this);
        }

        ~SubscriberNode() = default;

        static BT::PortsList providedPorts()
        {
            BT::PortsList ports { BT::InputPort<std::string>("topic", "Topic to subscribe"),
                                  BT::InputPort<uint32_t>("queue_size", 1, "Subscriber callback queue size"),
                                };

            const auto& policy_ports = SerializationPolicy<MessageType>::requiredPorts();
            ports.insert(policy_ports.cbegin(), policy_ports.cend());

            return ports;
        }

        virtual BT::NodeStatus tick() override
        {
            return BT::NodeStatus::SUCCESS;
        }

        virtual void halt() override {};

    private:
        void callback(const MessageType& _message)
        {
            serialization_policy_.onNewMessage(_message, *this);
        }

    private:
        ros::NodeHandle node_handle_;
        ros::Subscriber subscriber_;

        SerializationPolicy<MessageType> serialization_policy_ {};
};

//Shortcut alias
template <class MessageType>
using Subscriber = SubscriberNode<MessageType, NoSerialization>;

template <class MessageType>
using SerializedSubscriber = SubscriberNode<MessageType, JsonSerialization>;
}

#endif
