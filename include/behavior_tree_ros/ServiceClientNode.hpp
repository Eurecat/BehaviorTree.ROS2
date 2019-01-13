#ifndef SERVICE_CLIENT_NODE_HPP
#define SERVICE_CLIENT_NODE_HPP

#include "ROSActionNode.hpp"

namespace BT_ROS
{
template <class MessageType>
class ServiceClientNode final : public ROSActionNode
{
    public:
        ServiceClientNode(const std::string& _name, const NodeParameters& _params) : ROSActionNode(_name, _params)
        {}
        ~ServiceClientNode() = default;

        static const NodeParameters& requiredNodeParameters()
        {
            static BT::NodeParameters params { {"service", ""} };

            const auto& message_parameters = requiredMessageParameters<MessageType>();
            params.insert(message_parameters.cbegin(), message_parameters.cend());

            return params;
        }

        virtual BT::NodeStatus tick() override
        {
            try
            {
                auto message = buildMessage<MessageType>(*this);
                if(!client_.call(message)) { return NodeStatus::FAILURE; }
            }
            catch(const std::runtime_error&)      { return NodeStatus::FAILURE; }
            catch(const BT::bad_optional_access&) { return NodeStatus::FAILURE; }

            return NodeStatus::SUCCESS;
        }

        virtual void onInit() override
        {
            std::string service;
            if(!getParam("service", service)) { throw std::runtime_error {"Missing service name"}; }
            client_ = node_handle_.serviceClient<MessageType>(service);
        }

        virtual void halt() override {}

    private:
        ros::ServiceClient client_;
};

}

#endif
