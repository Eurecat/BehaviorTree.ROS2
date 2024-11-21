

//#include <mutex>
//#include <thread>

#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/deserialization_policies.hpp"
#include "behaviortree_ros2/serialization_policies.hpp"

//#include <std_srvs/srv/empty.hpp>

namespace BT
{

    template <class ServiceT, template <class> class RequestDeserializationPolicy,
                                template <class> class ResponseSerializationPolicy>
    class SerializedServiceNode : public RosServiceNode<ServiceT>,
                                    public RequestDeserializationPolicy<typename ServiceT::Request>,
                                    public ResponseSerializationPolicy<typename ServiceT::Response>
    {
        public:
            SerializedServiceNode(const std::string& _name, const BT::NodeConfig& conf, const BT::RosNodeParams& params) :
             RosServiceNode<ServiceT>(_name, conf, params){}
            ~SerializedServiceNode(){}
                
            static BT::PortsList providedPorts()
            {
                PortsList provided_port_list =  RosServiceNode<ServiceT>::providedPorts();

                const auto& request_ports = RequestDeserializationPolicy<typename ServiceT::Request>::requiredPorts();
                provided_port_list.insert(request_ports.cbegin(), request_ports.cend());

                const auto& response_ports = ResponseSerializationPolicy<typename ServiceT::Response>::requiredPorts();
                provided_port_list.insert(response_ports.cbegin(), response_ports.cend());

                return provided_port_list;
            }

            bool setRequest(typename ServiceT::Request::SharedPtr& request) override
            {
                //this->getInput("All_Data");
                //request->data = request_policy_.buildMessage(*this);
                return true;
            }

            BT::NodeStatus
            onResponseReceived(const typename ServiceT::Response::SharedPtr& response) override
            {
                std::cout << "onResponseReceived " << std::endl;
               /* if(response->success)
                {
                    response_policy_.onNewMessage(response, *this);
                    RCLCPP_INFO(this->logger(), "Service succeeded.");
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                   // RCLCPP_INFO(this->logger(), "Service failed: %s", response->message.c_str());
                    return BT::NodeStatus::FAILURE;
                }*/
                return BT::NodeStatus::SUCCESS;
            }

        private:

            RequestDeserializationPolicy<typename ServiceT::Request> request_policy_ {};
            ResponseSerializationPolicy<typename ServiceT::Response> response_policy_ {};
    };

    //Shortcut alias
    template <class ServiceT>
    using ServiceClient = SerializedServiceNode<ServiceT, BT_ROS::NoDeserialization, BT_ROS::NoSerialization>;

    template <class ServiceT>
    using AutomaticRequestServiceClient = SerializedServiceNode<ServiceT, BT_ROS::AutomaticDeserialization, BT_ROS::NoSerialization>;

    template <class ServiceT>
    using SerializedResponseServiceClient = SerializedServiceNode<ServiceT, BT_ROS::NoDeserialization, BT_ROS::JsonSerialization>;

    template <class ServiceT>
    using AutomaticServiceClient = SerializedServiceNode<ServiceT, BT_ROS::AutomaticDeserialization, BT_ROS::JsonSerialization>;

    template <class ServiceT>
    using AutomaticSmartServiceClient = SerializedServiceNode<ServiceT, BT_ROS::AutomaticDeserialization, BT_ROS::SmartJsonSerialization>;
}

