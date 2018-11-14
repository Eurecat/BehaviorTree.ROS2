#ifndef PUBLISHER_NODE_HPP
#define PUBLISHER_NODE_HPP

#include "ROSActionNode.hpp"

namespace BT_ROS
{
template <class MessageType>
std::vector<std::string> messageRequiredParameters();

template <class MessageType>
class PublisherNode final : public ROSActionNode
{
    public:
        PublisherNode(const std::string& _name, const BT::NodeParameters& _params) : ROSActionNode(_name, _params)
        {}
        ~PublisherNode() = default;

        static const BT::NodeParameters& requiredNodeParameters()
        {
            static BT::NodeParameters params { {"topic", ""} };

            const auto& message_parameters = messageRequiredParameters<MessageType>();
            for(const auto& param : message_parameters) { params.emplace(param, ""); }

            return params;
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);

            try
            {
                advertiseTopic();
                const auto& message = buildMessage<MessageType>(*this);
                publisher_->publish(message);
            }
            catch(const std::runtime_error&)      { return BT::NodeStatus::FAILURE; }
            catch(const BT::bad_optional_access&) { return BT::NodeStatus::FAILURE; }

            return BT::NodeStatus::SUCCESS;
        }

        virtual void halt() override {}

    private:
        void advertiseTopic()
        {
            if(publisher_) { return; }

            std::string topic;
            if(!getParam("topic", topic)) { throw std::runtime_error {"Missing topic name"}; }

            publisher_ = node_handle_.advertise<MessageType>(topic, 1);
        }

    private:
        BT::optional<ros::Publisher> publisher_;
};
}

#endif
