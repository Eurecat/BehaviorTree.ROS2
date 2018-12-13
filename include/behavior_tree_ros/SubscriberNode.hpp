#ifndef SUBSCRIBER_NODE_HPP
#define SUBSCRIBER_NODE_HPP

#include "ROSActionNode.hpp"

namespace BT_ROS
{
template <class MessageType>
class SubscriberNode final : public ROSActionNode
{
    public:
        SubscriberNode(const std::string& _name, const NodeParameters& _params) : ROSActionNode(_name, _params)
        {}
        ~SubscriberNode() = default;

        static const NodeParameters& requiredNodeParameters()
        {
            static BT::NodeParameters params { { "topic", "" }, { "queue_size", "1" }, { "key", "" } };
            return params;
        }

        virtual BT::NodeStatus tick() override
        {
            if(subscriber_) { return NodeStatus::SUCCESS; }

            setStatus(NodeStatus::RUNNING);

            try
            { 
                subscribeToTopic();
            }
            catch(const std::runtime_error&)      { setStatus(NodeStatus::FAILURE); }
            catch(const BT::bad_optional_access&) { setStatus(NodeStatus::FAILURE); }

            return status();
        }

        virtual void halt() override {}

    private:
        void subscribeToTopic()
        {
            std::string topic;
            uint32_t queue_size;
            if(!getParam("topic", topic))           { throw std::runtime_error {"Missing topic parameter"}; }
            if(!getParam("queue_size", queue_size)) { throw std::runtime_error {"Missing queue size parameter"}; }

            subscriber_ = node_handle_.subscribe(topic, queue_size, &SubscriberNode::callback, this);
        }

        void callback(const MessageType& _message)
        {
            blackboard()->set(getParam<std::string>("key").value(), _message);
        }

    private:
        BT::optional<ros::Subscriber> subscriber_;
};

}

#endif
