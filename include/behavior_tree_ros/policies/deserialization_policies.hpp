#ifndef DESERIALIZATION_POLICIES_HPP
#define DESERIALIZATION_POLICIES_HPP

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/utils/demangle_util.h>

#include "behavior_tree_ros/details/serialization.hpp"

namespace BT_ROS
{
template <class MessageType>
struct NoDeserialization
{
    static BT::PortsList requiredPorts()
    {
       if(serialization::isMsgEmpty<MessageType>()) { return {}; }

       return { BT::InputPort<MessageType>("input", "Input ROS message ["
                                            + BT::demangle(typeid(MessageType)) + "]") };
    }

    MessageType buildMessage(const BT::ActionNodeBase& _tree_node)
    {
       if(serialization::isMsgEmpty<MessageType>()) { return {}; }

        const auto& expected_message = _tree_node.getInput<MessageType>("input");
        if(!expected_message) { throw BT::RuntimeError { _tree_node.name() + ": " + expected_message.error() }; }

        return expected_message.value();
    }
};

template <class MessageType>
struct AutomaticDeserialization
{
    public:
        static BT::PortsList requiredPorts()
        {
            BT::PortsList ports {};

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

        MessageType buildMessage(const BT::ActionNodeBase& _tree_node)
        {
            serialization_buffer_.clear();

            try
            {
                for(const auto& field : msgInfo().fields())
                {
                    serialization::serializeField(_tree_node, field, serialization_buffer_);
                }

                MessageType message {};
                ros::serialization::IStream stream(serialization_buffer_.data(), serialization_buffer_.size());
                ros::serialization::Serializer<MessageType>::read(stream, message);

                return message;
            }
            catch(const std::out_of_range&)
            {
                throw BT::RuntimeError { _tree_node.name() + ": unrecognized field type in message " + BT::demangle(typeid(MessageType))
                                            + ". Non-builtin types automatic serialization is not supported."
                                            + " Use a different message creation policy." };
            }
        }

    private:
        static const RosIntrospection::ROSMessage& msgInfo() { static const auto msg_info = serialization::msgInfo<MessageType>(); return msg_info; };

    private:
        std::vector<uint8_t> serialization_buffer_;
};
}

#endif
