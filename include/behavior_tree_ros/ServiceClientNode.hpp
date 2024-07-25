#ifndef SERVICE_CLIENT_NODE_HPP
#define SERVICE_CLIENT_NODE_HPP

#include <mutex>
#include <thread>

#include <behaviortree_cpp_v3/action_node.h>

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
            client_instantiated_ = false;
            instantiateClient(false);
        }
        ~ServiceClientNode()
        {
            client_.shutdown();
            if (service_call_thread_.joinable())
            {
                service_call_thread_.join();
            }
        }

        void callService(const typename MessageType::Request& _request)
        {
            service_mutex_.lock();
            bool success = client_.call(_request, service_response_);
            service_state_ = success ? 2 : 1; // If service succedded to 2, otherwise to 1
            service_mutex_.unlock();
        }

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
            instantiateClient(true);
            setStatus(BT::NodeStatus::RUNNING);

            const auto& service_request = request_policy_.buildMessage(*this);
            // typename MessageType::Response service_response {};

            if (!client_.exists()) { return BT::NodeStatus::FAILURE; }

            // TODO: re-think thread things
            if(!service_called_)
            {
                service_call_thread_ = std::thread(&ServiceClientNode::callService, this, service_request);
                std::this_thread::sleep_for(std::chrono::milliseconds(200)); // sleep this thread for 200 ms
                service_called_ = true;
            }

            if(service_mutex_.try_lock())
            {
                if (service_state_ == 1) // service finishes with errors
                {
                    service_mutex_.unlock();
                    if (service_call_thread_.joinable()) { service_call_thread_.join(); }
                    service_state_ = 0;
                    service_called_ = false;
                    return BT::NodeStatus::FAILURE;
                }
                else if (service_state_ == 2) // service finishes successfully
                {
                    response_policy_.onNewMessage(service_response_, *this);
                    service_mutex_.unlock();
                    if (service_call_thread_.joinable()) { service_call_thread_.join(); }
                    service_state_ = 0;
                    service_called_ = false;
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    service_mutex_.unlock();
                }
            }

            return BT::NodeStatus::RUNNING;
        }

        virtual void halt() override
        {
            service_called_ = false;
            if (service_call_thread_.joinable())
            {
                service_call_thread_.detach();
            }
        }

    private:
        void instantiateClient(const bool mandatory)
        {
            if(client_instantiated_) return;
            
            const auto& service = getInput<std::string>("service");
            if(!service) 
            { 
                if(mandatory)
                    throw BT::RuntimeError { name() + ": " + service.error() }; 
                else
                    return;
            }

            client_ = node_handle_.serviceClient<MessageType>(service.value());
            client_instantiated_ = true;
        }

        ros::NodeHandle node_handle_;
        ros::ServiceClient client_;
        bool client_instantiated_;

        RequestDeserializationPolicy<typename MessageType::Request> request_policy_ {};
        ResponseSerializationPolicy<typename MessageType::Response> response_policy_ {};

        typename MessageType::Response service_response_ {};
        int service_state_ {};
        std::thread service_call_thread_;
        std::atomic<bool> service_called_ {false};
        std::mutex service_mutex_;
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

template <class MessageType>
using AutomaticSmartServiceClient = ServiceClientNode<MessageType, AutomaticDeserialization, SmartJsonSerialization>;
}

#endif
