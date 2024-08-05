#ifndef SUBSCRIBER_NODE_HPP
#define SUBSCRIBER_NODE_HPP

#include <mutex>
#include <chrono>
#include <behaviortree_cpp_v3/action_node.h>

#include "behavior_tree_ros/policies/serialization_policies.hpp"

namespace BT_ROS
{
template <class MessageType, template <class> class SerializationPolicy>
class SubscriberNode final : public BT::CoroActionNode, public SerializationPolicy<MessageType>
{
    public:
        SubscriberNode(const std::string& _name, const BT::NodeConfiguration& _config) : BT::CoroActionNode(_name, _config)
        {
            subscriber_initialized_ = false;
            fetchSubscriberValues(false);
        }

        ~SubscriberNode() = default;

        static BT::PortsList providedPorts()
        {
            BT::PortsList ports 
            { 
                BT::InputPort<std::string>("topic", "Topic to subscribe"),
                BT::InputPort<uint32_t>("queue_size", 1, "Subscriber callback queue size"),
                BT::InputPort<bool>("consume_msgs", false, "Should messages be consumed?"),
                BT::InputPort<bool>("reinit", false, "Instantiate the subscriber at every new tick"),
                BT::InputPort<uint32_t>("wait_ms_new_msg", 200, "How many ms you want to wait after instantiation to receive a new msg"),
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
            if(subscriber_ == nullptr || !subscriber_initialized_)
            {
                fetchSubscriberValues(true);
                subscriber_ = node_handle_.subscribe(topic_, queue_size_, &SubscriberNode::callback, this);
                start_waiting_time_ = std::chrono::system_clock::now();
                wait_duration_ = std::chrono::milliseconds{wait_ms_};
                // std::this_thread::sleep_for(std::chrono::milliseconds(200));
                while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_waiting_time_) < wait_duration_)
                {
                    setStatusRunningAndYield();
                }

            }

            std::lock_guard<std::mutex> lock (message_mutex_);

            bool new_message_written { false };

            if(message_)
            {
                serialization_policy_.onNewMessage(*message_, *this);
                message_.reset();
                new_message_written = true;
            }

            if(reinit_)// reinit?
            {
                subscriber_.shutdown();
                subscriber_initialized_ = false; 
            }

            // If messages are not expected to be consumed, return success only if at least one message has been received
            if(!consume_msgs_)
            { 
                return message_received_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            }

            // If not, return success only if a new message has been saved to the blackboard in the current tick
            return new_message_written ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }

        virtual void halt() override {CoroActionNode::halt();};

    private:
        void fetchSubscriberValues(const bool mandatory)
        {
            if(subscriber_initialized_) return;
            
            const auto& topic        = getInput<std::string>("topic"); // only required input with no default value

            // tunable input with default values
            const auto& queue_size   = getInput<uint32_t>("queue_size");
            const auto& consume_msgs = getInput<bool>("consume_msgs");
            const auto& reinit = getInput<bool>("reinit");
            const auto& wait_ms   = getInput<uint32_t>("wait_ms_new_msg");

            if(!topic && mandatory)        { throw BT::RuntimeError { name() + ": " + topic.error() };        }
            else if(!topic) return; // not mandatory

            consume_msgs_ = consume_msgs.value();
            topic_        = topic.value();
            queue_size_   = queue_size.value();
            reinit_ = reinit.value();
            wait_ms_ = wait_ms.value();

            subscriber_initialized_ = true;
        }

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

        bool subscriber_initialized_;

        std::chrono::system_clock::time_point start_waiting_time_;
        std::chrono::milliseconds wait_duration_;

        std::string topic_;
        uint32_t    queue_size_;
        bool        consume_msgs_;
        bool        reinit_;
        uint32_t    wait_ms_;

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

template <class MessageType>
using SmartSerializedSubscriber = SubscriberNode<MessageType, SmartJsonSerialization>;
}

#endif
