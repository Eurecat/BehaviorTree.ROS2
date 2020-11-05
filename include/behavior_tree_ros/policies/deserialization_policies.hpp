#ifndef DESERIALIZATION_POLICIES_HPP
#define DESERIALIZATION_POLICIES_HPP

#include <algorithm>
#include <functional>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/utils/demangle_util.h>

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

            const auto& field_ports = fieldPorts();

            for(const auto& field_port : field_ports)
            {
                // Time and duration defaults to ros::Time::now and zero
                // TODO: should we allow users to set this themselves? If so, how would they
                // write it? Maybe a custom convertFromString()?
                const auto type_id = field_port.second.type().typeID();
                if(type_id == RosIntrospection::TIME ||
                   type_id == RosIntrospection::DURATION) { continue; }

                const auto& port = serialization::getTypedPort(field_port.second,
                                                               BT::PortDirection::INPUT,
                                                               field_port.first,
                                                               std::string { "Auto-generated field from " }
                                                               + BT::demangle(typeid(MessageType)));
                ports.insert(port);
            }

            return ports;
        }

        MessageType buildMessage(const BT::ActionNodeBase& _tree_node)
        {
            serialization_buffer_.clear();

            const auto& field_ports = fieldPorts();

            try
            {
                for(const auto& field_port : field_ports)
                {
                    serialization::serializeField(_tree_node, field_port.first,
                                                  field_port.second.type().typeID(),
                                                  serialization_buffer_);
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
        // Matching between node port name and original ros message field
        using FieldPort = std::pair<std::string, RosIntrospection::ROSField>;
        static const std::vector<FieldPort>& fieldPorts()
        {
            static std::vector<FieldPort> field_ports;

            if(!field_ports.empty()) { return field_ports; }

            // I had to this recursively with a lambda instead of the same function
            // to be able to detect if the field_ports_ vector was already initialized
            std::function<void(const RosIntrospection::ROSMessage&, const std::string&)> recursive_gen;
            recursive_gen = [&](const RosIntrospection::ROSMessage& _msg, const std::string _prefix)
            {
                using namespace RosIntrospection;

                for(const ROSField& field : _msg.fields())
                {
                    // Skip constant fields
                    if(field.isConstant()) { continue; }

                    // If the field is not a built-in type, then find the message definition of that type and
                    // call this function again recursively to extract its built-in fields
                    if(!field.type().isBuiltin())
                    {
                        const auto& msg_list = msgInfo().type_list;

                        const auto msg_it = std::find_if(msg_list.cbegin(), msg_list.cend(), [&field]
                                            (const ROSMessage& _msg) { return _msg.type() == field.type(); });

                        recursive_gen(*msg_it, _prefix + field.name() + ".");
                        continue;
                    }

                    field_ports.emplace_back(_prefix + field.name(), field);
                }
            };

            const auto msg_tree_root = msgInfo().message_tree.croot();
            recursive_gen(*msg_tree_root->value(), "");

            return field_ports;
        }

        static const RosIntrospection::ROSMessageInfo& msgInfo()
        { 
            static RosIntrospection::Parser parser;
            parser.registerMessageDefinition(serialization::msgDataType<MessageType>(),
                                             serialization::msgType<MessageType>(),
                                             serialization::msgDefinition<MessageType>());
            
            static const auto msg_info = parser.getMessageInfo(serialization::msgDataType<MessageType>());
            return *msg_info;
        };

    private:
        std::vector<uint8_t> serialization_buffer_;
};

template <class MessageType>
struct CustomDeserialization
{
    static BT::PortsList requiredPorts() = delete;
    MessageType buildMessage(const BT::ActionNodeBase& _tree_node) = delete;
};

} // namespace BT_ROS

#endif
