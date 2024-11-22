#ifndef SERIALIZATION_POLICIES_HPP
#define SERIALIZATION_POLICIES_HPP

#include "behaviortree_ros2/parser_utils.hpp"

namespace BT_ROS

{
template <class MessageType>
struct EmptySerialization
{
    public:
        static BT::PortsList requiredPorts(const std::string& = {})
        {
            return {};
        }
        void initParser( std::string topic_name, std::string topic_type)
        {
            parser_init_ = true;
        }
        void onNewMessage(const std::shared_ptr<MessageType>&, BT::TreeNode&, const std::string& = {})
        {
            return;
        }
        bool isParserInit() {return parser_init_;}

    private:
        bool parser_init_{false};
};

template <class MessageType>
struct NoSerialization
{
    public:
        static BT::PortsList requiredPorts(const std::string& _port_name = "output")
        {
            if(isMsgEmpty<MessageType>()) { return {}; }

            return { BT::OutputPort<MessageType>(_port_name, "Received ROS message ["
                                                    + BT::demangle(typeid(MessageType)) + "]") };
        }
        void initParser( std::string topic_name, std::string topic_type)
        {
            parser_init_ = true;
        }
        void onNewMessage(const std::shared_ptr<MessageType>& _message, BT::TreeNode& _tree_node,
                        const std::string& _port_name = "output")
        {
            if(isMsgEmpty<MessageType>()) { return; }

            _tree_node.setOutput(_port_name, _message);
        }
        bool isParserInit() {return parser_init_;}
    private:
        bool parser_init_{false};
};

template <class MessageType>
struct JsonSerialization
{
    public:
        void initParser( std::string topic_name, std::string topic_type)
        {
            parser_ = std::make_shared<RosMsgParser::Parser>(topic_name, RosMsgParser::ROSType(topic_type), RosMsgParser::GetMessageDefinition(topic_type));
            parser_init_ = true;
            topic_type_ = topic_type;
        }
        static BT::PortsList requiredPorts(const std::string& _base_port_name = "output")
        {
          
            if(isMsgEmpty<MessageType>()) { return {}; }

            return { BT::OutputPort<nlohmann::json>("serialized_" + _base_port_name, "Serialized ROS message ["
                                                        + BT::demangle(typeid(MessageType)) + "]") };
        }
 
        void onNewMessage(const std::shared_ptr<MessageType>& _message, BT::TreeNode& _tree_node,
                          const std::string& _base_port_name = "output")
        {
            if(isMsgEmpty<MessageType>()) { return; }
 
            std::vector<uint8_t> buffer_in = RosMsgParser::BuildMessageBuffer(*(_message.get()), topic_type_);
            std::string json_text;
            RosMsgParser::ROS2_Deserializer deserializer_;
            parser_->deserializeIntoJson(buffer_in, &json_text, &deserializer_);
            nlohmann::json json_parsed = nlohmann::json::parse(json_text);
            _tree_node.setOutput("serialized_" + _base_port_name, json_parsed);
        }
        bool isParserInit() {return parser_init_;}
 
    private:
        std::string topic_type_;
        std::shared_ptr<RosMsgParser::Parser> parser_;

        bool parser_init_{false};
};

template <class MessageType>
struct SmartJsonSerialization
{
    public:
        SmartJsonSerialization(bool time_enabled = false) :
        time_enabled_serialization_(time_enabled)
        {
        }
        void initParser( std::string topic_name, std::string topic_type)
        {
            std::cout << "initParser" << topic_type << "\n" << std::flush;
            parser_ = std::make_shared<RosMsgParser::Parser>(topic_name, RosMsgParser::ROSType(topic_type), RosMsgParser::GetMessageDefinition(topic_type));
            parser_init_ = true;
            topic_type_ = topic_type;
        }
        // additional ports MessageType specific
        static void additionalPortsMessageSpecific(BT::PortsList& portsList){}

        // additional msg & json handling MessageType specific
        void useMsgBeforeSerialization(const std::shared_ptr<MessageType>& _message, BT::TreeNode& _tree_node){}
        void processMsgPostSerialization(const std::shared_ptr<MessageType>& _message, nlohmann::json& json, BT::TreeNode& _tree_node){}

        static BT::PortsList requiredPorts(const std::string& _base_port_name = "output")
        {
            if(isMsgEmpty<MessageType>()) { return {}; }

            BT::PortsList portsList = { 
                BT::InputPort<std::vector<std::string>>("ignore_fields", {}, "Fields to be ignored in the serialization"),
                BT::OutputPort<nlohmann::json>("serialized_" + _base_port_name, "Serialized ROS message ["+ BT::demangle(typeid(MessageType)) + "]") 
            };
            additionalPortsMessageSpecific(portsList);
            return portsList;
        }

        void onNewMessage(const std::shared_ptr<MessageType>& _message, BT::TreeNode& _tree_node,
                          const std::string& _base_port_name = "output")
        {
            if(isMsgEmpty<MessageType>()) { return; }
    
            useMsgBeforeSerialization(_message, _tree_node);
            
            const std::vector<std::string> ignore_fields = _tree_node.getInput<std::vector<std::string>>("ignore_fields").value_or(std::vector<std::string>{});

            if (!time_enabled_serialization_)
            {
                //TODO: Add Time Field to ignore_fields
            }

            std::vector<uint8_t> buffer_in = RosMsgParser::BuildMessageBuffer(*(_message.get()), topic_type_);
            std::string json_text;
            RosMsgParser::ROS2_Deserializer deserializer_;
            parser_->deserializeIntoJson(buffer_in, &json_text, &deserializer_, 0, false, ignore_fields);
            nlohmann::json json_parsed = nlohmann::json::parse(json_text);

            processMsgPostSerialization(_message, json_parsed, _tree_node);

            _tree_node.setOutput("serialized_" + _base_port_name, json_parsed);
        }
        bool isParserInit() {return parser_init_;}

    private:

        bool time_enabled_serialization_;
        std::shared_ptr<RosMsgParser::Parser> parser_;

        bool parser_init_{false};
        std::string topic_type_;
};

template <class MessageType>
struct SmartTimeEnabledJsonSerialization : public SmartJsonSerialization<MessageType>
{
    public:
        SmartTimeEnabledJsonSerialization() : SmartJsonSerialization<MessageType>(true) {};
};

template <class MessageType>
struct CustomSerialization
{
    static BT::PortsList requiredPorts(const std::string& = {}) = delete;
    void onNewMessage(const std::shared_ptr<MessageType>&, BT::TreeNode&, const std::string& = {}) = delete;
};

} // namespace BT_ROS

#endif
