#ifndef SERVICE_CLIENT_NODE_HPP
#define SERVICE_CLIENT_NODE_HPP

#include <behaviortree_cpp/action_node.h>

#include "behavior_tree_ros/policies/deserialization_policies.hpp"
#include "behavior_tree_ros/policies/serialization_policies.hpp"

namespace BT_ROS
{
template <class MessageType, template <class> class RequestDeserializationPolicy,
                             template <class> class ResponseSerializationPolicy>
class ServiceClientNode final : public BT::ActionNodeBase,
                                public RequestDeserializationPolicy<typename MessageType::Request>,
                                public ResponseSerializationPolicy<typename MessageType::Response>
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

            const auto& request_ports = RequestDeserializationPolicy<typename MessageType::Request>::requiredPorts();
            ports.insert(request_ports.cbegin(), request_ports.cend());

            const auto& response_ports = ResponseSerializationPolicy<typename MessageType::Response>::requiredPorts();
            ports.insert(response_ports.cbegin(), response_ports.cend());

            return ports;
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);

            const auto& service_request = request_policy_.buildMessage(*this);
            typename MessageType::Response service_response {};

            if(!client_.call(service_request, service_response)) { return BT::NodeStatus::FAILURE; }

            response_policy_.onNewMessage(service_response, *this);
            return BT::NodeStatus::SUCCESS;
        }

        virtual void halt() override {}

    private:
        ros::NodeHandle node_handle_;
        ros::ServiceClient client_;

        RequestDeserializationPolicy<typename MessageType::Request> request_policy_ {};
        ResponseSerializationPolicy<typename MessageType::Response> response_policy_ {};
};

//Shortcut alias
template <class MessageType>
using ServiceClient = ServiceClientNode<MessageType, NoDeserialization, NoSerialization>;

template <class MessageType>
using AutomaticRequestServiceClient = ServiceClientNode<MessageType, AutomaticDeserialization, NoSerialization>;

template <class MessageType>
using SerializedResponseServiceClient = ServiceClientNode<MessageType, NoDeserialization, JsonSerialization>;

template <class MessageType>
using AutomaticServiceClient = ServiceClientNode<MessageType, AutomaticDeserialization, JsonSerialization>;
}

#endif
