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
            const auto& service = getInput<std::string>("service");
            if(!service) { throw BT::RuntimeError { name() + ": " + service.error() }; }

            client_ = node_handle_.serviceClient<MessageType>(service.value());
        }
        ~ServiceClientNode()
        {
            client_.shutdown();
            if (service_call_thread_.joinable())
            {
                printf("joining thread\n");
                service_call_thread_.join();
            }
            else
            {
                printf("detaching thread\n");
                service_call_thread_.detach();
            }
        }

        void callService(const typename MessageType::Request& _request, typename MessageType::Response& _response)
        {
            std::lock_guard<std::mutex> lock (service_mutex_);
            bool success = client_.call(_request, _response);
            service_state_ = success ? 2 : 1; // If service succedded to 2, otherwise to 1
            // service_mutex_.unlock();
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
            setStatus(BT::NodeStatus::RUNNING);

            const auto& service_request = request_policy_.buildMessage(*this);
            typename MessageType::Response service_response {};

            if (!client_.exists()) { return BT::NodeStatus::FAILURE; }

            if(!service_called_)
            {
                service_call_thread_ = std::thread(&ServiceClientNode::callService, this, std::ref(service_request), std::ref(service_response));
                std::this_thread::sleep_for(std::chrono::milliseconds(200)); // sleep this thread for 200 ms
                service_called_ = true;
            }

            if(service_mutex_.try_lock())
            {
                if (service_state_ == 1)
                {
                    service_mutex_.unlock();
                    service_call_thread_.join();
                    service_state_ = 0;
                    service_called_ = false;
                    return BT::NodeStatus::FAILURE;
                }
                else if (service_state_ == 2)
                {
                    response_policy_.onNewMessage(service_response, *this);
                    service_mutex_.unlock();
                    service_call_thread_.join();
                    service_state_ = 0;
                    service_called_ = true;
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    service_mutex_.unlock();
                }
            }

            return BT::NodeStatus::RUNNING;
        }

        virtual void halt() override {}

    private:
        ros::NodeHandle node_handle_;
        ros::ServiceClient client_;

        RequestDeserializationPolicy<typename MessageType::Request> request_policy_ {};
        ResponseSerializationPolicy<typename MessageType::Response> response_policy_ {};

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
}

#endif
