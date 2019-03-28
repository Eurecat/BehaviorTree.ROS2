#ifndef PUBLISHER_NODE_HPP
#define PUBLISHER_NODE_HPP

#include <behaviortree_cpp/utils/demangle_util.h>

#include "ROSActionNode.hpp"
#include "details/serialization.hpp"

//TODO: use a better approach to support user-defined parsing functions (like policies for example)
namespace BT_ROS
{
template <class MessageType>
class PublisherNode final : public ROSActionNode
{
    public:
        PublisherNode(const std::string& _name, const BT::NodeConfiguration& _config) : ROSActionNode(_name, _config)
        {
            const auto& topic      = getInput<std::string>("topic");
            const auto& queue_size = getInput<uint32_t>("queue_size");
            const auto& latch      = getInput<bool>("latch");

            if(!topic)      { throw BT::RuntimeError { name() + ": " + topic.error() };      }
            if(!queue_size) { throw BT::RuntimeError { name() + ": " + queue_size.error() }; }
            if(!latch)      { throw BT::RuntimeError { name() + ": " + latch.error() };      }

            publisher_ = node_handle_.advertise<MessageType>(topic.value(), queue_size.value(), latch.value());
        }
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
                //Setting void as the port type disables type checking
                const auto& field_port = BT::InputPort<void>(field.name(), std::string { "Auto-generated field from " }
                                                                            + BT::demangle(typeid(MessageType)));
                ports.insert(field_port);
            }

            return ports;
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);

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

                return BT::NodeStatus::SUCCESS;
            }
            catch(const std::out_of_range&)
            {
                throw BT::RuntimeError { name() + ": unrecognized field type in message " +  std::string { serialization::msgDataType<MessageType>() }
                                            + ". Non-builtin types automatic serialization is not supported."
                                            + " Implement specializations for buildMessage<> and requiredMessageParameters<> functions instead." };
            }
        }

        virtual void halt() override {}

    private:
        static const RosIntrospection::ROSMessage& msgInfo() { static const auto msg_info = serialization::msgInfo<MessageType>(); return msg_info; }

    private:
        ros::Publisher publisher_;

        std::vector<uint8_t> serialization_buffer_;
};
}

#endif
