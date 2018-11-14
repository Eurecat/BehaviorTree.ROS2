#ifndef SERVICE_CLIENT_NODE_HPP
#define SERVICE_CLIENT_NODE_HPP

#include "ROSActionNode.hpp"

namespace BT_ROS
{
template <class MessageType>
std::vector<std::string> messageRequiredParameters();

template <class MessageType>
class ServiceClientNode final : public ROSActionNode
{
    public:
        ServiceClientNode(const std::string& _name, const BT::NodeParameters& _params) : ROSActionNode(_name, _params)
        {}
        ~ServiceClientNode() = default;

        static const BT::NodeParameters& requiredNodeParameters()
        {
            static BT::NodeParameters params { {"service", ""} };

            const auto& message_parameters = messageRequiredParameters<MessageType>();
            for(const auto& param : message_parameters) { params.emplace(param, ""); }

            return params;
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);

            try
            {
                connectToService();
                auto message = buildMessage<MessageType>(*this);
                if(!client_->call(message)) { return BT::NodeStatus::FAILURE; }
            }
            catch(const std::runtime_error&)      { return BT::NodeStatus::FAILURE; }
            catch(const BT::bad_optional_access&) { return BT::NodeStatus::FAILURE; }

            return BT::NodeStatus::SUCCESS;
        }

        virtual void halt() override {}

    private:
        void connectToService()
        {
            if(client_) { return; }

            std::string service;
            if(!getParam("service", service)) { throw std::runtime_error {"Missing service name"}; }
            client_ = node_handle_.serviceClient<MessageType>(service);
        }

    private:
        BT::optional<ros::ServiceClient> client_;
};

}

#endif
