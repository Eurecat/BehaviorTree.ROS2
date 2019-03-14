#ifndef PUBLISHER_NODE_HPP
#define PUBLISHER_NODE_HPP

#include "ROSActionNode.hpp"
#include "details/serialization.hpp"

namespace BT_ROS
{
template <class MessageType>
class BasePublisherNode : public ROSActionNode
{
    public:
        using ROSActionNode::ROSActionNode;
        virtual ~BasePublisherNode() = default;

        /*
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
        */

        virtual void halt() override {}

    protected:
        ros::Publisher publisher_;
};

template <class MessageType, bool Serialize = true>
class PublisherNode;

template <class MessageType>
class PublisherNode<MessageType, false> final : public BasePublisherNode<MessageType>
{
    public:
        using BasePublisherNode<MessageType>::BasePublisherNode;
        ~PublisherNode() = default;

        static BT::PortsList providedPorts()
        {
            BT::PortsList ports { BT::InputPort<std::string>("topic", "Topic to publish to"),
                                  BT::InputPort<uint32_t>("queue_size", 1, "Internal publisher queue size"),
                                  BT::InputPort<bool>("latch", false, "Latch messages?")
                                };
            
            const auto& message_ports = requiredMessagePorts<MessageType>();
            ports.insert(message_ports.cbegin(), message_ports.cend());

            return ports;
        }

        virtual BT::NodeStatus tick() override
        {
            try
            {
                const auto& message = buildMessage<MessageType>(*this);
                this->publisher_.publish(message);
            }
            catch(const std::runtime_error&)      { return BT::NodeStatus::FAILURE; }
            //catch(const BT::bad_optional_access&) { return BT::NodeStatus::FAILURE; }

            return BT::NodeStatus::SUCCESS;
        }
};

template <class MessageType>
class PublisherNode<MessageType, true> final : public BasePublisherNode<MessageType>
{
    public:
        using BasePublisherNode<MessageType>::BasePublisherNode;
        ~PublisherNode() = default;

        static BT::PortsList providedPorts()
        {
            BT::PortsList ports { BT::InputPort<std::string>("topic", "Topic to publish to"),
                                  BT::InputPort<uint32_t>("queue_size", 1, "Internal publisher queue size"),
                                  BT::InputPort<bool>("latch", false, "Latch messages?")
                                };

            for(const auto& field : msgInfo().fields())
            {
                if(field.isConstant()) { continue; }
                //TODO: get correct type
                const auto& field_port = BT::InputPort<std::string>(field.name(), "Field test");
                ports.insert(field_port);
            }

            return ports;
        }

        virtual BT::NodeStatus tick() override
        {
            try
            {
                for(const auto& field : msgInfo().fields())
                {
                    serialization::serializeField(*this, field, serialization_buffer_);
                }

                //Note: is it possible to transmit the serialized data directly?
                MessageType message;
                ros::serialization::IStream stream(serialization_buffer_.data(), serialization_buffer_.size());
                ros::serialization::Serializer<MessageType>::read(stream, message);
                this->publisher_.publish(message);
                serialization_buffer_.clear();
            }
            catch(const std::runtime_error&)      { return BT::NodeStatus::FAILURE; }
            //catch(const BT::bad_optional_access&) { return BT::NodeStatus::FAILURE; }
            catch(const std::out_of_range&)
            {
                throw std::runtime_error { "PublisherNode: unrecognized field type in message " +  std::string { serialization::msgDataType<MessageType>() }
                                            + ". Non-builtin types automatic serialization is not supported."
                                            + " Implement specializations for buildMessage<> and requiredMessageParameters<> functions instead." };
            }

            return BT::NodeStatus::SUCCESS;
        }

    private:
        static const RosIntrospection::ROSMessage& msgInfo() { static const auto msg_info = serialization::msgInfo<MessageType>(); return msg_info; }

    private:
        std::vector<uint8_t> serialization_buffer_;
};
}

#endif
