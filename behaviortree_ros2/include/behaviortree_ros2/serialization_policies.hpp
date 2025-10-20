#ifndef SERIALIZATION_POLICIES_HPP
#define SERIALIZATION_POLICIES_HPP

#include "behaviortree_ros2/parser_utils.hpp"
#include "behaviortree_eut_plugins/utils/deserialize_json.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

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
struct AutomaticSerialization
{
    public:
        void initParser( std::string topic_name, std::string topic_type)
        {
            parser_ = std::make_shared<RosMsgParser::Parser>(topic_name, RosMsgParser::ROSType(topic_type), RosMsgParser::GetMessageDefinition(topic_type));
            parser_init_ = true;
            topic_type_ = topic_type;
        }

        static BT::PortsList requiredPorts(const std::string& _base_port_name = "")
        {
            BT::PortsList ports {};
            if(isMsgEmpty<MessageType>()) { return ports; }
            const auto& field_ports = fieldPorts<MessageType>({RosMsgParser::ROSType("builtin_interfaces/Time")});
            for(const auto& field_port : field_ports)
            {
                // Time and duration defaults to ros::Time::now and zero
                // TODO should we allow users to set this themselves? If so, how would they
                // write it? Maybe a custom convertFromString()?
                const auto type_id = field_port.second.type().typeID();
                if(type_id == RosMsgParser::TIME ||
                type_id == RosMsgParser::DURATION) { continue; }
                
                std::string port_name = _base_port_name.empty() ? field_port.first : _base_port_name + "_" + field_port.first;
                const auto& port = getTypedPort(field_port.second,BT::PortDirection::OUTPUT, port_name, std::string { "Auto-generated field from " } + BT::demangle(typeid(MessageType)));

                ports.insert(port);
            }
            return ports;
        }

        // Helper to extract nested fields "header.frame_id" [header][frame_id]
        nlohmann::json getNestedValue(const nlohmann::json& j, const std::string& dotted_key)
        {
            std::istringstream iss(dotted_key);
            std::string token;
            const nlohmann::json* current = &j;
            while (std::getline(iss, token, '.'))
            {
                if (!current->contains(token))
                    throw std::runtime_error("Missing key: " + token + " in " + dotted_key);
                current = &((*current)[token]);
            }

            return *current;
        }

        void onNewMessage( const std::shared_ptr<MessageType>& _message, BT::TreeNode& _tree_node, const std::string& _base_port_name = "")
        {
            if(isMsgEmpty<MessageType>()) { return; }
        
            std::vector<uint8_t> buffer_in = RosMsgParser::BuildMessageBuffer(*(_message.get()), topic_type_);
            std::string json_text;
            RosMsgParser::ROS2_Deserializer deserializer_;
            parser_->deserializeIntoJson(buffer_in, &json_text, &deserializer_, 0, true, true);
            nlohmann::json json_parsed = nlohmann::json::parse(json_text);
        
            const auto& field_ports = fieldPorts<MessageType>({RosMsgParser::ROSType("builtin_interfaces/Time")});
        
            for(const auto& field_port : field_ports)
            {
                const auto type_id = field_port.second.type().typeID();
                if(type_id == RosMsgParser::TIME || type_id == RosMsgParser::DURATION)
                    continue;
        
                std::string port_name = _base_port_name.empty() 
                    ? field_port.first 
                    : _base_port_name + "_" + field_port.first;
        
                try
                {
                    nlohmann::json value = getNestedValue(json_parsed, field_port.first);
        
                    BT::EutUtils::deserializeField(_tree_node, port_name, value);
        
                }
                catch(const std::exception& e)
                {
                    std::cerr << "Warning: could not deserialize field " << field_port.first 
                              << ": " << e.what() << std::endl;
                }
        
            }
        }

        bool isParserInit() {return parser_init_;}
 
    private:
        std::string topic_type_;
        std::shared_ptr<RosMsgParser::Parser> parser_;
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
            parser_->deserializeIntoJson(buffer_in, &json_text, &deserializer_, 0, true, true);
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
            parser_ = std::make_shared<RosMsgParser::Parser>(topic_name, RosMsgParser::ROSType(topic_type), RosMsgParser::GetMessageDefinition(topic_type));
            parser_init_ = true;
            topic_type_ = topic_type;
        }
        // additional ports MessageType specific
        static void additionalPortsMessageSpecific(BT::PortsList& portsList){}

        // additional msg & json handling MessageType specific
        void useMsgBeforeSerialization(const std::shared_ptr<MessageType> _message, BT::TreeNode& _tree_node){}
        void processMsgPostSerialization(const std::shared_ptr<MessageType> _message, nlohmann::json& json, BT::TreeNode& _tree_node){}

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
            parser_->deserializeIntoJson(buffer_in, &json_text, &deserializer_, 0, true, true, ignore_fields);
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

