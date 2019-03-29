#ifndef SERIALIZATION_POLICIES_HPP
#define SERIALIZATION_POLICIES_HPP

#include <behaviortree_cpp/action_node.h>

#include "behavior_tree_ros/details/serialization.hpp"

namespace BT_ROS
{
template <class MessageType>
struct NoSerialization
{
    static BT::PortsList requiredPorts()
    {
       return { BT::OutputPort<MessageType>("output", "Received ROS message ["
                                                + BT::demangle(typeid(MessageType)) + "]") };
    }

    void onNewMessage(const MessageType& _message, BT::ActionNodeBase& _tree_node)
    {
        _tree_node.setOutput("output", _message);
    }
};

template <class MessageType>
struct JsonSerialization
{
    public:
        JsonSerialization()
        {
            parser().registerMessageDefinition(serialization::msgDataType<MessageType>(),
                                               serialization::msgType<MessageType>(),
                                               serialization::msgDefinition<MessageType>());
        }

        static BT::PortsList requiredPorts()
        {
            return { BT::OutputPort<nlohmann::json>("serialized_output", "Serialized ROS message ["
                                                        + BT::demangle(typeid(MessageType)) + "]") };
        }

        void onNewMessage(const MessageType& _message, BT::ActionNodeBase& _tree_node)
        {
            buffer_.resize(ros::serialization::serializationLength(_message));
            ros::serialization::OStream stream(buffer_.data(), buffer_.size());
            ros::serialization::serialize(stream, _message);

            parser().deserializeIntoFlatContainer(serialization::msgDataType<MessageType>(),
                                                  absl::Span<uint8_t>(buffer_), &flat_message_, buffer_.size());

            //Serialization is done in to_json() function (serialization.hpp)
            nlohmann::json serialized_json = flat_message_;
            _tree_node.setOutput("serialized_output", serialized_json);
        }

    private:
        static RosIntrospection::Parser& parser() { static RosIntrospection::Parser parser; return parser; };

    private:
        RosIntrospection::FlatMessage flat_message_;
        std::vector<uint8_t> buffer_;
};
}

#endif
