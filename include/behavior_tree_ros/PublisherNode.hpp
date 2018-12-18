#ifndef PUBLISHER_NODE_HPP
#define PUBLISHER_NODE_HPP

#include <behaviortree_cpp/basic_types.h>

#include "ROSActionNode.hpp"

namespace BT_ROS
{
template <class MessageType>
class PublisherNode final : public ROSActionNode
{
    public:
        PublisherNode(const std::string& _name, const NodeParameters& _params) : ROSActionNode(_name, _params)
        {}
        ~PublisherNode() = default;

        static const NodeParameters& requiredNodeParameters()
        {
            static NodeParameters params { { "topic", "" }, { "queue_size", "1" }, { "latch", "false" } };

            const auto& message_parameters = requiredMessageParameters<MessageType>();
            params.insert(message_parameters.cbegin(), message_parameters.cend());

            return params;
        }

        virtual NodeStatus tick() override
        {
            setStatus(NodeStatus::RUNNING);

            try
            {
                const auto& message = buildMessage<MessageType>(*this);
                publisher_.publish(message);
            }
            catch(const std::runtime_error&)      { return NodeStatus::FAILURE; }
            catch(const BT::bad_optional_access&) { return NodeStatus::FAILURE; }

            return NodeStatus::SUCCESS;
        }

        virtual void onInit() override
        {
            std::string topic;
            uint32_t queue_size;
            bool latch;

            if(!getParam("topic", topic))           { throw std::runtime_error { "Missing topic parameter" }; }
            if(!getParam("queue_size", queue_size)) { throw std::runtime_error { "Missing queue size parameter" }; }
            if(!getParam("latch", latch))           { throw std::runtime_error { "Missing latch parameter" }; }

            publisher_ = node_handle_.advertise<MessageType>(topic, queue_size, latch);
        }

        virtual void halt() override {}

    private:
        ros::Publisher publisher_;
};
}

#endif
