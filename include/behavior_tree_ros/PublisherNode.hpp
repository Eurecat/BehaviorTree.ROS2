#ifndef PUBLISHER_NODE_HPP
#define PUBLISHER_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>

#include "behavior_tree_ros/policies/deserialization_policies.hpp"

namespace BT_ROS
{
template <class MessageType, template <class> class DeserializationPolicy>
class PublisherNode final : public BT::ActionNodeBase, public DeserializationPolicy<MessageType>
{
    public:
        PublisherNode(const std::string& _name, const BT::NodeConfiguration& _config) : ActionNodeBase(_name, _config)
        {
            advertisePublisher(false); //do not trigger a fatal failure if you don't have the possibility to advertise topic now, i.e. instantiate publisher
        }
        ~PublisherNode() = default;

        static BT::PortsList providedPorts()
        {
            BT::PortsList ports { BT::InputPort<std::string>("topic", "Topic to publish to"),
                                  BT::InputPort<uint32_t>("queue_size", 1, "Internal publisher queue size"),
                                  BT::InputPort<bool>("latch", false, "Latch messages?"),
                                  BT::InputPort<int32_t>("wait_subscribers", -1, "Wait a certain number of subscribers before publishing. If -1, default behavior, don't wait")
                                };

            const auto& field_ports = DeserializationPolicy<MessageType>::requiredPorts();
            ports.insert(field_ports.cbegin(), field_ports.cend());

            return ports;
        }

        virtual BT::NodeStatus tick() override
        {
            advertisePublisher(true); //advertise publisher if you haven't done it in the constructor
            setStatus(BT::NodeStatus::RUNNING);
            const int32_t wait_subs = getInput<int32_t>("wait_subscribers").value_or(-1);
            if(wait_subs > 0 && wait_subs > static_cast<int32_t>(publisher_.getNumSubscribers()))
            {
                const auto& topic      = getInput<std::string>("topic");
                ROS_INFO("sub to %s are %d", topic.value().c_str(), publisher_.getNumSubscribers());
                //PROBLEM: FIRST MESSAGE IS ALWAYS LOST when advertizing and publishing instantly
                //add sleep to avoid issues when using a ros::AsyncSpinner
                // std::this_thread::sleep_for(std::chrono::milliseconds(200));
                return BT::NodeStatus::RUNNING;
            }

            const auto& message = deserialization_policy_.buildMessage(*this);
            publisher_.publish(message);
            return BT::NodeStatus::SUCCESS;
        }

        virtual void halt() override {}

    private:
        void advertisePublisher(const bool mandatory)
        {
            const auto& topic      = getInput<std::string>("topic");
            if(publisher_.getTopic().empty() ||                                 // publisher never set up
                (topic.has_value() && topic.value() != publisher_.getTopic()))  // new topic
            {
                const auto& queue_size = getInput<uint32_t>("queue_size");
                const auto& latch      = getInput<bool>("latch");
                
                if(mandatory)
                {
                    if(!topic)      { throw BT::RuntimeError { name() + ": " + topic.error() };      }
                    if(!queue_size) { throw BT::RuntimeError { name() + ": " + queue_size.error() }; }
                    if(!latch)      { throw BT::RuntimeError { name() + ": " + latch.error() };      }
                    
                    publisher_ = node_handle_.advertise<MessageType>(topic.value(), queue_size.value(), latch.value());

                }
                else if(!topic || !queue_size || !latch) 
                {
                    return;
                }
                else {
                    publisher_ = node_handle_.advertise<MessageType>(topic.value(), queue_size.value(), latch.value());
                }
            }
        }

        ros::NodeHandle node_handle_;
        ros::Publisher publisher_;

        DeserializationPolicy<MessageType> deserialization_policy_ {};
};

//Shortcut alias
template <class MessageType>
using Publisher = PublisherNode<MessageType, NoDeserialization>;

template <class MessageType>
using AutomaticPublisher = PublisherNode<MessageType, AutomaticDeserialization>;
}

#endif
