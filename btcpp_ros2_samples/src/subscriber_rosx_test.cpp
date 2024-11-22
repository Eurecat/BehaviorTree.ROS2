#include "behaviortree_ros2/bt_utils.hpp"
#include "rosx_introspection/ros_parser.hpp"
#include "rosx_introspection/ros_utils/ros2_helpers.hpp"
#include "std_msgs/msg/bool.hpp" 
using namespace BT;

// Simple tree, used to execute once each action.
static const char* xml_publisher_text = R"(
  <root BTCPP_format="4">
    <BehaviorTree>
      <Sequence>
        <PublishStdEmpty topic_name="/emptyyy" name="B"/>
        <PublishStdShort topic_name="/shooort" data="1" name="A"/>
        <MonitorStdBool topic_name="/asdf" name="D"/>
      </Sequence>
    </BehaviorTree>
  </root>
 )";

 static const char* xml_subscriber_text = R"(
  <root BTCPP_format="4">
    <BehaviorTree>
      <Sequence>
        <MonitorStdBool topic_name="/asdf" name="D"/>
        <MonitorStdEmpty topic_name="/emptyyy" name="B"/>
        <MonitorStdShort topic_name="/shooort" data="1" name="A"/>
      </Sequence>
    </BehaviorTree>
  </root>
 )";

 static const char* xml_service_text = R"(
  <root BTCPP_format="4">
    <BehaviorTree>
      <Sequence>
        <PublishStdBool topic_name="/booool" data="true" name="A"/>
        <CallSetBoolService service_name="robotA/set_bool" data="false" name="callerservice"/>
        <CallSetBoolService service_name="robotA/set_bool" data="true" name="callerservice"/>
        <MonitorStdEmpty topic_name="/emptyyy" name="B"/>
      </Sequence>
    </BehaviorTree>
  </root>
 )";
 static const char* xml_action_text = R"(
  <root BTCPP_format="4">
    <BehaviorTree>
      <Sequence>
        <TestActionSleep action_name="/sleep_service" msec_timeout="2000" name="sleepA"/>
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
  RCLCPP_INFO(nh->get_logger(),"Start plugin registration");
  //Register with plugin
  bt_server::Params bt_params;
  bt_params.ros_plugins_timeout = 1000;
  nh->declare_parameter("plugins_dir","behaviortree_ros2/bt_plugins");
  std::string plugin_directory = nh->get_parameter("plugins_dir").as_string();
  RCLCPP_INFO(nh->get_logger(),"Got directory: %s",plugin_directory.c_str());
  bt_params.plugins.push_back(plugin_directory);
  for(const auto& plugin : bt_params.plugins)
  {
    RCLCPP_INFO(nh->get_logger(),"Added directory %s",plugin.c_str());
  }
  RegisterPlugins(bt_params, factory, nh);
  RCLCPP_INFO(nh->get_logger(),"Registered OK");
  //Register without plugin
  //factory.registerNodeType<ReceiveString>("ReceiveString", params);

  auto tree = factory.createTreeFromText(xml_action_text);
  RCLCPP_INFO(nh->get_logger(),"Created OK");
  while(rclcpp::ok())
  {
    tree.tickWhileRunning();
  }

  return 0;
}
