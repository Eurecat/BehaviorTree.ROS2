#ifndef SUBSCRIBER_NODE_HPP
#define SUBSCRIBER_NODE_HPP

#include <mutex>
#include <behaviortree_cpp/action_node.h>

#include "behavior_tree_ros/policies/serialization_policies.hpp"

namespace BT_ROS
{
template <class MessageType, template <class> class SerializationPolicy>
class SubscriberNode final : public BT::ActionNodeBase, public SerializationPolicy<MessageType>
{
    public:
        SubscriberNode(const std::string& _name, const BT::NodeConfiguration& _config) : ActionNodeBase(_name, _config)
        {
            const auto& topic        = getInput<std::string>("topic");
            const auto& queue_size   = getInput<uint32_t>("queue_size");
            const auto& consume_msgs = getInput<bool>("consume_msgs");

            if(!topic)        { throw BT::RuntimeError { name() + ": " + topic.error() };        }
            if(!queue_size)   { throw BT::RuntimeError { name() + ": " + queue_size.error() };   }
            if(!consume_msgs) { throw BT::RuntimeError { name() + ": " + consume_msgs.error() }; }

            consume_msgs_ = consume_msgs.value();
            topic_        = topic.value();
            queue_size_   = queue_size.value();
        }

        ~SubscriberNode() = default;

        static BT::PortsList providedPorts()
        {
            BT::PortsList ports { BT::InputPort<std::string>("topic", "Topic to subscribe"),
                BT::InputPort<uint32_t>("queue_size", 1, "Subscriber callback queue size"),
                BT::InputPort<bool>("consume_msgs", false, "Should messages be consumed?"),
            };

            const auto& policy_ports = SerializationPolicy<MessageType>::requiredPorts();
            ports.insert(policy_ports.cbegin(), policy_ports.cend());

            return ports;
        }

        // TODO: would it be better to somehow call the ROS callback queue manually from here?
        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);

            //Subscribe if not already subscribed (this is done here instead of the constructor
            //to avoid issues when using a ros::AsyncSpinner)
            if(subscriber_ == nullptr)
            {
                subscriber_ = node_handle_.subscribe(topic_, queue_size_, &SubscriberNode::callback, this);
            }

            std::lock_guard<std::mutex> lock (message_mutex_);

            bool new_message_written { false };

            if(message_)
            {
                serialization_policy_.onNewMessage(*message_, *this);
                message_.reset();
                new_message_written = true;
            }

            // If messages are not expected to be consumed, return success only if at least one message has been received
            if(!consume_msgs_)
            { 
                return message_received_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            }

            // If not, return success only if a new message has been saved to the blackboard in the current tick
            return new_message_written ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }

        virtual void halt() override {};

    private:
        //TODO: let users choose thread policies (aka do not assume that this is running in a different thread)
        void callback(const typename MessageType::Ptr& _message)
        {
            std::lock_guard<std::mutex> lock (message_mutex_);

            message_ = _message;
            message_received_ = true; 
        }

    private:
        ros::NodeHandle node_handle_;
        ros::Subscriber subscriber_;

        std::string topic_;
        uint32_t    queue_size_;
        bool        consume_msgs_;

        typename MessageType::Ptr message_ {};
        bool message_received_ {};
        std::mutex message_mutex_;

        SerializationPolicy<MessageType> serialization_policy_ {};
};

//Shortcut alias
template <class MessageType>
using Subscriber = SubscriberNode<MessageType, NoSerialization>;

template <class MessageType>
using SerializedSubscriber = SubscriberNode<MessageType, JsonSerialization>;
}

#endif
