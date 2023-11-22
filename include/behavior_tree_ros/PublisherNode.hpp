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
            advertisePublisherIfNeeded(false); //do not trigger a fatal failure if you don't have the possibility to advertise topic now, i.e. instantiate publisher
        }
        ~PublisherNode() = default;

        static BT::PortsList providedPorts()
        {
            BT::PortsList ports { BT::InputPort<std::string>("topic", "Topic to publish to"),
                                  BT::InputPort<uint32_t>("queue_size", 1, "Internal publisher queue size"),
                                  BT::InputPort<bool>("latch", false, "Latch messages?")
                                };

            const auto& field_ports = DeserializationPolicy<MessageType>::requiredPorts();
            ports.insert(field_ports.cbegin(), field_ports.cend());

            return ports;
        }

        virtual BT::NodeStatus tick() override
        {
            advertisePublisherIfNeeded(true); //advertise publisher if you haven't done it in the constructor
            setStatus(BT::NodeStatus::RUNNING);

            const auto& message = deserialization_policy_.buildMessage(*this);
            publisher_.publish(message);
            return BT::NodeStatus::SUCCESS;
        }

        virtual void halt() override {}

    private:
        void advertisePublisherIfNeeded(const bool fatal_failure)
        {
            if(publisher_.getTopic().empty())
            {
                const auto& topic      = getInput<std::string>("topic");
                const auto& queue_size = getInput<uint32_t>("queue_size");
                const auto& latch      = getInput<bool>("latch");
                
                if(fatal_failure)
                {
                    if(!topic)      { throw BT::RuntimeError { name() + ": " + topic.error() };      }
                    if(!queue_size) { throw BT::RuntimeError { name() + ": " + queue_size.error() }; }
                    if(!latch)      { throw BT::RuntimeError { name() + ": " + latch.error() };      }
                }
                else if(!topic || !queue_size || !latch) return;

                publisher_ = node_handle_.advertise<MessageType>(topic.value(), queue_size.value(), latch.value());
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
