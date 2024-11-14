#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <std_msgs/msg/string.hpp>
#include "behaviortree_ros2/plugins.hpp"

// rosx_introspection tests
#include "rosx_introspection/ros_parser.hpp"
#include "rosx_introspection/ros_utils/ros2_helpers.hpp"

using namespace BT;
using namespace RosMsgParser;

class ReceiveString : public RosTopicSubNode<std_msgs::msg::String>
{
public:
  ReceiveString(const std::string& name, const NodeConfig& conf,
                const RosNodeParams& params)
    : RosTopicSubNode<std_msgs::msg::String>(name, conf, params)
  {
    const std::string topic_type = "std_msgs/String";

    parser_ = std::make_shared<RosMsgParser::Parser>(topic_name_, ROSType(topic_type),
                          GetMessageDefinition(topic_type)); 
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("topic_name")};
  }

  NodeStatus onTick(const std::shared_ptr<std_msgs::msg::String>& last_msg) override
  {
    if(last_msg)  // empty if no new message received, since the last tick
    {
      RCLCPP_INFO(logger(), "[%s] new message: %s", name().c_str(),
                  last_msg->data.c_str());

      std::vector<uint8_t> buffer_in = BuildMessageBuffer(*(last_msg.get()), "std_msgs/String");
      RosMsgParser::FlatMessage flat_container;
      parser_->deserialize(Span<uint8_t>(buffer_in), &flat_container, &deserializer_);
      for (auto& it : flat_container.value)
      {
        if(it.second.getTypeID() == BuiltinType::STRING)
          std::cout << it.first << " >> " << it.second.convert<std::string>() << std::endl;
      }

      for (auto& it : flat_container.name)
      {
        std::cout << it.first << " >> " << it.second << std::endl;
      }

      std::string json_text;
      parser_->deserializeIntoJson(buffer_in, &json_text, &deserializer_);

      std::cout << "\n JSON encoding [std_msgs/String]:\n" << json_text << std::endl;

      // test round-robin transform
      ROS2_Serializer serializer;
      parser_->serializeFromJson(json_text, &serializer);

      auto std_msgs_string_out = BufferToMessage<std_msgs::msg::String>(
        serializer.getBufferData(), serializer.getBufferSize()
      );

      std::cout << "\n JSON decoding [std_msgs/String]:\n" << std_msgs_string_out.data << std::endl;
    }
    return NodeStatus::SUCCESS;
  }

  private:
    std::shared_ptr<RosMsgParser::Parser> parser_;
    ROS2_Deserializer deserializer_;

};

// Plugin registration:
//CreateRosNodePlugin(ReceiveString, "ReceiveString");

class ReceiveString2 : public RosTopicSubNode<std_msgs::msg::String>
{
public:
  ReceiveString2(const std::string& name, const NodeConfig& conf,
                const RosNodeParams& params)
    : RosTopicSubNode<std_msgs::msg::String>(name, conf, params)
  {
    const std::string topic_type = "std_msgs/String";

    parser_ = std::make_shared<RosMsgParser::Parser>(topic_name_, ROSType(topic_type),
                          GetMessageDefinition(topic_type)); 
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("topic_name")};
  }

  NodeStatus onTick(const std::shared_ptr<std_msgs::msg::String>& last_msg) override
  {
    if(last_msg)  // empty if no new message received, since the last tick
    {
      RCLCPP_INFO(logger(), "[%s] new message: %s", name().c_str(),
                  last_msg->data.c_str());

      std::vector<uint8_t> buffer_in = BuildMessageBuffer(*(last_msg.get()), "std_msgs/String");
      RosMsgParser::FlatMessage flat_container;
      parser_->deserialize(Span<uint8_t>(buffer_in), &flat_container, &deserializer_);
      for (auto& it : flat_container.value)
      {
        if(it.second.getTypeID() == BuiltinType::STRING)
          std::cout << it.first << " >> " << it.second.convert<std::string>() << std::endl;
      }

      for (auto& it : flat_container.name)
      {
        std::cout << it.first << " >> " << it.second << std::endl;
      }

      std::string json_text;
      parser_->deserializeIntoJson(buffer_in, &json_text, &deserializer_);

      std::cout << "\n JSON encoding [std_msgs/String]:\n" << json_text << std::endl;

      // test round-robin transform
      ROS2_Serializer serializer;
      parser_->serializeFromJson(json_text, &serializer);

      auto std_msgs_string_out = BufferToMessage<std_msgs::msg::String>(
        serializer.getBufferData(), serializer.getBufferSize()
      );

      std::cout << "\n JSON decoding [std_msgs/String]:\n" << std_msgs_string_out.data << std::endl;
    }
    return NodeStatus::SUCCESS;
  }

  private:
    std::shared_ptr<RosMsgParser::Parser> parser_;
    ROS2_Deserializer deserializer_;

};
//CreateRosNodePlugin(ReceiveString2, "ReceiveString2");

BT_REGISTER_ROS_NODES(factory, params)
{
   factory.registerNodeType<ReceiveString>("ReceiveString", params);
   factory.registerNodeType<ReceiveString2>("ReceiveString2", params);
   factory.registerNodeType<ReceiveString2>("ReceiveString3", params);
};