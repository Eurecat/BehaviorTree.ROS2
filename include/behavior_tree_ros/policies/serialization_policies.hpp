#ifndef SERIALIZATION_POLICIES_HPP
#define SERIALIZATION_POLICIES_HPP

#include <behaviortree_cpp_v3/action_node.h>

#include "behavior_tree_ros/details/serialization.hpp"

namespace BT_ROS
{
template <class MessageType>
struct EmptySerialization
{
    static BT::PortsList requiredPorts(const std::string& = {})
    {
        return {};
    }

    void onNewMessage(const MessageType&, BT::ActionNodeBase&, const std::string& = {})
    {
        return;
    }
};

template <class MessageType>
struct NoSerialization
{
    static BT::PortsList requiredPorts(const std::string& _port_name = "output")
    {
        if(serialization::isMsgEmpty<MessageType>()) { return {}; }

        return { BT::OutputPort<MessageType>(_port_name, "Received ROS message ["
                                                + BT::demangle(typeid(MessageType)) + "]") };
    }

    void onNewMessage(const MessageType& _message, BT::ActionNodeBase& _tree_node,
                      const std::string& _port_name = "output")
    {
        if(serialization::isMsgEmpty<MessageType>()) { return; }

        _tree_node.setOutput(_port_name, _message);
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
 
        static BT::PortsList requiredPorts(const std::string& _base_port_name = "output")
        {
            if(serialization::isMsgEmpty<MessageType>()) { return {}; }
 
            return { BT::OutputPort<nlohmann::json>("serialized_" + _base_port_name, "Serialized ROS message ["
                                                        + BT::demangle(typeid(MessageType)) + "]") };
        }
 
        void onNewMessage(const MessageType& _message, BT::ActionNodeBase& _tree_node,
                          const std::string& _base_port_name = "output")
        {
            if(serialization::isMsgEmpty<MessageType>()) { return; }
 
            buffer_.resize(ros::serialization::serializationLength(_message));
            ros::serialization::OStream stream(buffer_.data(), buffer_.size());
            ros::serialization::serialize(stream, _message);
 
            bool flat_deserialize_result = parser().deserializeIntoFlatContainer(serialization::msgDataType<MessageType>(),
                                                  RosIntrospection::Span<uint8_t>(buffer_), &flat_message_, buffer_.size());
 
            //Serialization is done in to_json() function (serialization.hpp)
            nlohmann::json serialized_json = flat_deserialize_result? nlohmann::FlatMessageWithIgnoredFields(flat_message_) : nlohmann::json{};
            _tree_node.setOutput("serialized_" + _base_port_name, serialized_json);
        }
 
    private:
        static RosIntrospection::Parser& parser() { static RosIntrospection::Parser parser; return parser; };
 
    private:
        RosIntrospection::FlatMessage flat_message_;
        std::vector<uint8_t> buffer_;
};

template <class MessageType>
struct SmartJsonSerialization
{
    public:
        SmartJsonSerialization()
        {
            parser().registerMessageDefinition(serialization::msgDataType<MessageType>(),
                                               serialization::msgType<MessageType>(),
                                               serialization::msgDefinition<MessageType>());
        }

        // additional ports MessageType specific
        static void additionalPortsMessageSpecific(BT::PortsList& portsList){}

        // additional msg & json handling MessageType specific
        void useMsgBeforeSerialization(const MessageType& _message, BT::ActionNodeBase& _tree_node){}
        void processMsgPostSerialization(const MessageType& _message, nlohmann::json& json, BT::ActionNodeBase& _tree_node){}

        static BT::PortsList requiredPorts(const std::string& _base_port_name = "output")
        {
            if(serialization::isMsgEmpty<MessageType>()) { return {}; }

            BT::PortsList portsList = { 
                BT::InputPort<std::vector<std::string>>("ignore_fields", {}, "Fields to be ignored in the serialization"),
                BT::OutputPort<nlohmann::json>("serialized_" + _base_port_name, "Serialized ROS message ["+ BT::demangle(typeid(MessageType)) + "]") 
            };
            additionalPortsMessageSpecific(portsList);
            return portsList;
        }

        void onNewMessage(const MessageType& _message, BT::ActionNodeBase& _tree_node,
                          const std::string& _base_port_name = "output")
        {
            if(serialization::isMsgEmpty<MessageType>()) { return; }
    
            useMsgBeforeSerialization(_message, _tree_node);
            
            const std::vector<std::string> ignore_fields = _tree_node.getInput<std::vector<std::string>>("ignore_fields").value_or(std::vector<std::string>{});
           

            buffer_.resize(ros::serialization::serializationLength(_message));
            ros::serialization::OStream stream(buffer_.data(), buffer_.size());
            ros::serialization::serialize(stream, _message);

            bool flat_deserialize_result = parser().deserializeIntoFlatContainer(serialization::msgDataType<MessageType>(),
                                                  RosIntrospection::Span<uint8_t>(buffer_), &flat_message_, buffer_.size());

            //Serialization is done in to_json() function (serialization.hpp)
            nlohmann::json serialized_json = flat_deserialize_result? nlohmann::FlatMessageWithIgnoredFields(flat_message_, ignore_fields) : nlohmann::json{};
            processMsgPostSerialization(_message, serialized_json, _tree_node);
            _tree_node.setOutput("serialized_" + _base_port_name, serialized_json);
        }

    private:
        static RosIntrospection::Parser& parser() { static RosIntrospection::Parser parser; parser.setMaxArrayPolicy(RosIntrospection::Parser::MaxArrayPolicy::KEEP_LARGE_ARRAYS); return parser; };

    private:
        RosIntrospection::FlatMessage flat_message_;
        std::vector<uint8_t> buffer_;
};

template <class MessageType>
struct CustomSerialization
{
    static BT::PortsList requiredPorts(const std::string& = {}) = delete;
    void onNewMessage(const MessageType&, BT::ActionNodeBase&, const std::string& = {}) = delete;
};

} // namespace BT_ROS

#endif
