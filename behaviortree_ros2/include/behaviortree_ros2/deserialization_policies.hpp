#ifndef DESERIALIZATION_POLICIES_HPP
#define DESERIALIZATION_POLICIES_HPP

#include "behaviortree_ros2/parser_utils.hpp"
#include "behavior_tree_eut_plugins/eut_debug.h"
namespace BT_ROS
{
    template <class MessageType>
    struct NoDeserialization
    {
        public:
            static BT::PortsList requiredPorts()
            {
                if(isMsgEmpty<MessageType>()) { return {}; }

                return { BT::InputPort<MessageType>("input", "Input ROS message ["
                                                    + BT::demangle(typeid(MessageType)) + "]") };
            }

            MessageType buildMessage(const BT::TreeNode& _tree_node)
            {
                if(isMsgEmpty<MessageType>()) { return {}; }

                const auto& expected_message = _tree_node.getInput<MessageType>("input");
                if(!expected_message) { throw BT::RuntimeError { _tree_node.name() + ": " + expected_message.error() }; }

                return expected_message.value();
            }
            void initParser( std::string topic_name, std::string topic_type)
            {
                parser_init_ = true;
            }
            bool isParserInit() {return parser_init_;}
        private:
            bool parser_init_{false};
    };

    template <class MessageType>
    struct AutomaticDeserialization
    {
        public:
            static BT::PortsList requiredPorts()
            {
                BT::PortsList ports {};
                if(isMsgEmpty<MessageType>()) { return ports; }
                const auto& field_ports = fieldPorts();
                for(const auto& field_port : field_ports)
                {
                    // Time and duration defaults to ros::Time::now and zero
                    // TODO should we allow users to set this themselves? If so, how would they
                    // write it? Maybe a custom convertFromString()?
                    const auto type_id = field_port.second.type().typeID();
                    if(type_id == RosMsgParser::TIME ||
                    type_id == RosMsgParser::DURATION) { continue; }
                    
                    const auto& port = getTypedPort(field_port.second,BT::PortDirection::INPUT, field_port.first, std::string { "Auto-generated field from " } + BT::demangle(typeid(MessageType)));

                    ports.insert(port);
                }

                return ports;
            }

            MessageType buildMessage(const BT::TreeNode& _tree_node)
            {
                MessageType ros_message {};
                if(isMsgEmpty<MessageType>()) { return ros_message; }

                const auto& field_ports = fieldPorts();
              
                try
                {
                    RosMsgParser::ROS2_Serializer serializer_;
                    nlohmann::json result = buildJson(field_ports, _tree_node);
                    parser_->serializeFromJson(result.dump(), &serializer_);
                    ros_message = RosMsgParser::BufferToMessage<MessageType>( serializer_.getBufferData(), serializer_.getBufferSize() );
                }
                catch(const std::out_of_range&)
                {
                    throw BT::RuntimeError { _tree_node.name() + ": unrecognized field type in message " + BT::demangle(typeid(MessageType))
                                                + ". Non-builtin types automatic serialization is not supported."
                                                + " Use a different message creation policy." };
                }
                return ros_message;
            }

            void initParser( std::string topic_name, std::string topic_type)
            {
                parser_ = std::make_shared<RosMsgParser::Parser>(topic_name, RosMsgParser::ROSType(topic_type), RosMsgParser::GetMessageDefinition(topic_type));
                parser_init_ = true;
                topic_type_ = topic_type;
            }
            bool isParserInit() {return parser_init_;}
        private:
            // Matching between node port name and original ros message field
            using FieldPort = std::pair<std::string, RosMsgParser::ROSField>;
            static const std::vector<FieldPort>& fieldPorts()
            {
                static std::vector<FieldPort> field_ports;

                if(!field_ports.empty()) { return field_ports; }

                // I had to this recursively with a lambda instead of the same function
                // to be able to detect if the field_ports_ vector was already initialized
                std::function<void(const RosMsgParser::ROSMessage&, const std::string&)> recursive_gen;
                recursive_gen = [&](const RosMsgParser::ROSMessage& _msg, const std::string _prefix)
                {
                    using namespace RosMsgParser;
                    //std::cout << "fieldPorts() for msg type " << BT::demangle(typeid(MessageType)) << " size = " << _msg.fields().size() << "\n" << std::flush;
                    for(const ROSField& field : _msg.fields())
                    {
                        // Skip constant fields
                        if(field.isConstant()) { continue; }
                        
                        // If the field is not a built-in type, then find the message definition of that type and
                        // call this function again recursively to extract its built-in fields
                         if (!field.type().isBuiltin())
                            {
                                auto msg_ptr = msgInfo()->msg_library.at(field.type());
                                recursive_gen(*msg_ptr, _prefix + field.name() + ".");
                                continue;
                            }

                        // Rename "ID" and "name" ports to avoid conflict with keywords. Adding "_" in the front
                        std::string port_name = ( _prefix.empty() && (field.name() == "ID" || field.name() == "name") ) ? "_" + field.name() :
                                                                                                                        field.name();
                        
                        field_ports.emplace_back(_prefix + port_name, field);
                    }
                };

                const auto msg_tree_root = msgInfo()->root_msg;
                recursive_gen(*msg_tree_root, "");

                return field_ports;
            }

            static const std::shared_ptr<RosMsgParser::MessageSchema>& msgInfo()
            { 
                std::string topic_type = msgName<MessageType>();
                std::shared_ptr<RosMsgParser::Parser> parser;
                parser = std::make_shared<RosMsgParser::Parser>("root", RosMsgParser::ROSType(topic_type), RosMsgParser::GetMessageDefinition(topic_type));
                static const auto msg_info = parser->getSchema();
                return msg_info;
            };

            
            nlohmann::json buildJson(const std::vector<FieldPort>& ports,const BT::TreeNode& tree_node) {
                nlohmann::json result;
                for (const auto& port : ports) {
                    
                    std::string name = port.first;
                    nlohmann::json* current = &result;
                    // Split the name by '.'
                    size_t pos = 0;
                    while ((pos = name.find('.')) != std::string::npos) {
                        std::string key = name.substr(0, pos);
                        name = name.substr(pos + 1);

                        // Navigate into the nested structure or create it
                        if (!current->contains(key)) {
                            (*current)[key] = nlohmann::json::object();
                        }
                        current = &(*current)[key];
                    }

                    auto portValue = BT::getPortValueAsJson(tree_node, port.first, BT::PortDirection::INPUT);

                    // Check if the Expected contains a valid value or an error
                    if (portValue.has_value()) {
                        // Assign the valid JSON value to the current nested structure
                        (*current)[name] = portValue.value();
                    } else {
                        // Handle the error case - for example, log it or assign a default value
                        std::cerr << "Error: " << portValue.error() << " for port " << name << std::endl;
                        (*current)[name] = nullptr;  // You could assign a default value, e.g., null or an empty object
                    }
                }
                return result;
            }

            std::string topic_type_;
            std::shared_ptr<RosMsgParser::Parser> parser_;
           
            bool parser_init_{false};
    };

    template <class MessageType>
    struct CustomDeserialization
    {
        static BT::PortsList requiredPorts() = delete;
        MessageType buildMessage(const BT::TreeNode& _tree_node) = delete;
    };

} // namespace BT_ROS

#endif
