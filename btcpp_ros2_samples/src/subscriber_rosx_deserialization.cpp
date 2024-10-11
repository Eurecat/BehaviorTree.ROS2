#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <std_msgs/msg/string.hpp>

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
    return {};
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

// Simple tree, used to execute once each action.
static const char* xml_text = R"(
  <root BTCPP_format="4">
    <BehaviorTree>
      <Sequence>
        <ReceiveString name="A"/>
        <ReceiveString name="B"/>
        <ReceiveString name="C"/>
      </Sequence>
    </BehaviorTree>
  </root>
 )";

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("subscriber_test");

  BehaviorTreeFactory factory;

  RosNodeParams params;
  params.nh = nh;
  params.default_port_value = "btcpp_string";
  factory.registerNodeType<ReceiveString>("ReceiveString", params);

  auto tree = factory.createTreeFromText(xml_text);

  while(rclcpp::ok())
  {
    tree.tickWhileRunning();
  }

  return 0;
}
