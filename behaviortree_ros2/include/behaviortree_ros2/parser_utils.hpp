#ifndef PARSER_UTILS_HPP
#define PARSER_UTILS_HPP

#include "rosx_introspection/ros_parser.hpp"
#include "rosx_introspection/ros_utils/ros2_helpers.hpp"

namespace BT_ROS
{
  // e.g. std_msgs::msg::Bool
  template <class MessageType>
  inline const char* msgDataType()
  {
    static std::string datatype = rosidl_generator_traits::data_type<MessageType>();
    return datatype.c_str();;
  };

  // e.g. std_msgs/msg/Bool
  template <class MessageType>
  inline const char* msgName()
  {
    return rosidl_generator_traits::name<MessageType>();
  }

  template <class MessageType>
  inline bool isMsgEmpty()
  {
    std::string msgDef = RosMsgParser::GetMessageDefinition(msgName<MessageType>());
    if (msgDef.empty() || msgDef == "\n") {
      return true;
    }
    return false;
  };

  using PortData     = std::pair<std::string, BT::PortInfo>;
  using SetPortFunction = std::function<PortData(const BT::PortDirection, const std::string&, const std::string&)>;

  static const std::unordered_map<RosMsgParser::BuiltinType, SetPortFunction> generate_port_map
  {
      { RosMsgParser::UINT8,    [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<uint8_t>(_direction, _name, _description);  }},
      { RosMsgParser::UINT16,   [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<uint16_t>(_direction, _name, _description); }},
      { RosMsgParser::UINT32,   [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<uint32_t>(_direction, _name, _description); }},
      { RosMsgParser::UINT64,   [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<uint64_t>(_direction, _name, _description); }},
      { RosMsgParser::BOOL,     [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<bool>(_direction, _name, _description);     }},
      { RosMsgParser::BYTE,     [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<int8_t>(_direction, _name, _description);   }},
      { RosMsgParser::CHAR,     [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<uint8_t>(_direction, _name, _description);  }},
      { RosMsgParser::INT8,     [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<int8_t>(_direction, _name, _description);   }},
      { RosMsgParser::INT16,    [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<int16_t>(_direction, _name, _description);  }},
      { RosMsgParser::INT32,    [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<int32_t>(_direction, _name, _description);  }},
      { RosMsgParser::INT64,    [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<int64_t>(_direction, _name, _description);  }},
      { RosMsgParser::FLOAT32,  [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<float>(_direction, _name, _description);    }},
      { RosMsgParser::FLOAT64,  [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<double>(_direction, _name, _description);   }},
      { RosMsgParser::STRING,   [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<std::string>(_direction, _name, _description); }},
      //{ RosMsgParser::TIME,   [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<rclcpp::Time>(_direction, _name, _description); }},
      //{ RosMsgParser::DURATION,   [] (const auto _direction, const auto& _name, const auto& _description) { return BT::CreatePort<rclcpp::Duration>(_direction, _name, _description); }},
  };

  inline PortData getTypedPort(const RosMsgParser::ROSField& _field,
                                const BT::PortDirection _port_direction,
                                const std::string& _port_name,
                                const std::string& _port_description)
  {
      return generate_port_map.at(_field.type().typeID())(_port_direction, _port_name, _port_description);
  }

  template <class MessageType>
  static const std::shared_ptr<RosMsgParser::MessageSchema>& msgInfo()
  { 
      std::string topic_type = msgName<MessageType>();
      std::shared_ptr<RosMsgParser::Parser> parser;
      parser = std::make_shared<RosMsgParser::Parser>("root", RosMsgParser::ROSType(topic_type), RosMsgParser::GetMessageDefinition(topic_type));
      static const auto msg_info = parser->getSchema();
      return msg_info;
  };
  
  using FieldPort = std::pair<std::string, RosMsgParser::ROSField>;

  template <class MessageType>
  static const std::vector<FieldPort>& fieldPorts(const std::unordered_set<RosMsgParser::ROSType>& ignore_types = {})
  {
      static std::vector<FieldPort> field_ports;
      
      if(!field_ports.empty()) { return field_ports; }

      // I had to this recursively with a lambda instead of the same function
      // to be able to detect if the field_ports_ vector was already initialized
      std::function<void(const RosMsgParser::ROSMessage&, const std::string&)> recursive_gen;
      recursive_gen = [&](const RosMsgParser::ROSMessage& _msg, const std::string _prefix)
      {
          using namespace RosMsgParser;
          for(const ROSField& field : _msg.fields())
          {
              // Skip constant fields
              if(field.isConstant()) { continue; }

              // Skip ignore types fields
              if(ignore_types.count(field.type())) { continue; }
              
              // If the field is not a built-in type, then find the message definition of that type and
              // call this function again recursively to extract its built-in fields
              if (!field.type().isBuiltin())
                  {
                      auto msg_ptr = msgInfo<MessageType>()->msg_library.at(field.type());
                      recursive_gen(*msg_ptr, _prefix + field.name() + ".");
                      continue;
                  }

              // Rename "ID" and "name" ports to avoid conflict with keywords. Adding "_" in the front
              std::string port_name = ( _prefix.empty() && (field.name() == "ID" || field.name() == "name") ) ? "_" + field.name() :
                                                                                                              field.name();
              
              field_ports.emplace_back(_prefix + port_name, field);
          }
      };

      const auto msg_tree_root = msgInfo<MessageType>()->root_msg;
      recursive_gen(*msg_tree_root, "");

      return field_ports;
  }


}

#endif