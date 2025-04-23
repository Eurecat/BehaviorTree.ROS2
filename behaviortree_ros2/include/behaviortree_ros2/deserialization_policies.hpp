#ifndef DESERIALIZATION_POLICIES_HPP
#define DESERIALIZATION_POLICIES_HPP

#include "behaviortree_ros2/parser_utils.hpp"
#include "behaviortree_eut_plugins/utils/eut_utils.h"

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
                MessageType ros_message {};
                if(isMsgEmpty<MessageType>()) { return ros_message; }

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
                const auto& field_ports = fieldPorts<MessageType>({RosMsgParser::ROSType("builtin_interfaces/Time")});
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

                const auto& field_ports = fieldPorts<MessageType>({RosMsgParser::ROSType("builtin_interfaces/Time")});
              
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
                catch(const std::invalid_argument& ex)
                {
                    throw BT::RuntimeError { _tree_node.name() + ": Invalid argument serialization while trying to serialize message " + BT::demangle(typeid(MessageType))
                                                + ": " + ex.what() };
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
            nlohmann::json buildJson(const std::vector<FieldPort>& ports,const BT::TreeNode& tree_node) {
                nlohmann::json result;
                for (const auto& port : ports) {

                    const std::string& full_field_name = port.first;
                    std::string rel_field_name = port.first;
                    nlohmann::json* current = &result;
                    
                    // Split the name by '.'
                    size_t pos = 0;
                    while ((pos = rel_field_name.find('.')) != std::string::npos) {
                        std::string key = rel_field_name.substr(0, pos);
                        rel_field_name = rel_field_name.substr(pos + 1);

                        // Navigate into the nested structure or create it
                        if (!current->contains(key)) {
                            (*current)[key] = nlohmann::json::object();
                        }
                        current = &(*current)[key];
                    }
                    
                    auto portValue = BT::EutUtils::getPortValueAsJson(tree_node, full_field_name, BT::PortDirection::INPUT);

                    // Check if the Expected contains a valid value or an error
                    if (portValue.has_value()) {
                        // Assign the valid JSON value to the current nested structure
                        (*current)[rel_field_name] = portValue.value();
                    } else {
                        // Handle the error case - for example, log it or assign a default value
                        std::cerr << "Error: " << portValue.error() << " for port " << rel_field_name << std::endl;
                        (*current)[rel_field_name] = nullptr;  // You could assign a default value, e.g., null or an empty object
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
