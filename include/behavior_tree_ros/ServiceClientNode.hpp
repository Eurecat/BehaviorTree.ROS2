#ifndef SERVICE_CLIENT_NODE_HPP
#define SERVICE_CLIENT_NODE_HPP

#include <behaviortree_cpp/action_node.h>

#include "behavior_tree_ros/policies/deserialization_policies.hpp"
#include "behavior_tree_ros/details/conversion_types.hpp"

namespace BT_ROS
{
template <class MessageType, template <class> class RequestDeserializationPolicy>
class ServiceClientNode final : public BT::ActionNodeBase, public RequestDeserializationPolicy<typename MessageType::Request>
{
    public:
        ServiceClientNode(const std::string& _name, const BT::NodeConfiguration& _config) : ActionNodeBase(_name, _config)
        {
            const auto& service = getInput<std::string>("service");
            if(!service) { throw BT::RuntimeError { name() + ": " + service.error() }; }

            client_ = node_handle_.serviceClient<MessageType>(service.value());
        }
        ~ServiceClientNode() = default;

        static BT::PortsList providedPorts()
        {
            BT:: PortsList ports { BT::InputPort<std::string>("service", "ROS service name") };

            const auto& field_ports = RequestDeserializationPolicy<typename MessageType::Request>::requiredPorts();
            ports.insert(field_ports.cbegin(), field_ports.cend());

            return ports;
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);

            const auto& service_request = request_policy_.buildMessage(*this);
            typename MessageType::Response service_response {};

            //TODO: do something with the response. Probably an option to either serialize it or store it
            //as a message, like how it's done with the subscriber
            if(!client_.call(service_request, service_response)) { return BT::NodeStatus::FAILURE; }

            return BT::NodeStatus::SUCCESS;
        }

        virtual void halt() override {}

    private:
        ros::NodeHandle node_handle_;
        ros::ServiceClient client_;

        RequestDeserializationPolicy<typename MessageType::Request> request_policy_ {};
};

//Shortcut alias
template <class MessageType>
using ServiceClient = ServiceClientNode<MessageType, NoDeserialization>;

template <class MessageType>
using CustomServiceClient = ServiceClientNode<MessageType, CustomDeserialization>;

template <class MessageType>
using AutomaticServiceClient = ServiceClientNode<MessageType, AutomaticDeserialization>;
}

#endif
