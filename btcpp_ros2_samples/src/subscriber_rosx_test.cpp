#include "behaviortree_ros2/bt_utils.hpp"
#include "rosx_introspection/ros_parser.hpp"
#include "rosx_introspection/ros_utils/ros2_helpers.hpp"
#include "std_msgs/msg/bool.hpp" 
using namespace BT;

// Simple tree, used to execute once each action.
static const char* xml_text = R"(
  <root BTCPP_format="4">
    <BehaviorTree>
      <Sequence>
        <PublishStdEmpty topic_name="/emptyyy" name="B"/>
        <PublishStdUChar topic_name="/uchaaar" data="0" name="B"/>
        <PublishStdChar topic_name="/chaar" data="5" name="B"/>
        <PublishStdShort topic_name="/shooort" data="1" name="A"/>
        <PublishStdInt topic_name="/inttt" data="-2" name="B"/>
        <PublishStdUInt topic_name="/uinttt" data="3" name="C"/>
        <MonitorStdBool topic_name="/asdf" name="D"/>
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

  auto tree = factory.createTreeFromText(xml_text);
  RCLCPP_INFO(nh->get_logger(),"Created OK");
  while(rclcpp::ok())
  {
    tree.tickWhileRunning();
  }

  return 0;
}
