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
      return RosMsgParser::GetMessageDefinition(msgName<MessageType>()).empty();
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
}

#endif