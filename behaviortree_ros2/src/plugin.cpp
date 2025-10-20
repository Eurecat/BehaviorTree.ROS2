#include "behaviortree_ros2/behaviortree_ros2.hpp"

#include "behaviortree_ros2/actions/ros_log.hpp"
#include "behaviortree_ros2/actions/GetTransformAnglesNode.hpp"
#include "behaviortree_ros2/actions/GetTransformDistanceNode.hpp"
#include "behaviortree_ros2/actions/GetTransformHorizontalDistance.hpp"
#include "behaviortree_ros2/actions/LookupTransformNode.hpp"

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
   
//TEST
#include <std_msgs/msg/int8_multi_array.hpp>
#include "btcpp_ros2_interfaces/action/sleep.hpp"
#include "btcpp_ros2_interfaces/action/execute_tree.hpp"
#include "btcpp_ros2_interfaces/msg/custom_msg.hpp"

// Simple Action that updates an instance of Position2D in the blackboard
class SayHi : public BT::StatefulActionNode
{
public:
  SayHi(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("person_name") };
  }

  BT::NodeStatus onStart() override
  {
    const auto name = getInput<std::string>("person_name");
    if(!name)
      return BT::NodeStatus::FAILURE;

    person_name_ = name.value();
    counter_ = 0;

    std::cout << "Starting to say hi to " << person_name_ << "..." << std::endl;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (counter_ < 3)
    {
      std::cout << "Hi " << person_name_ << "! (" << counter_+1 << "/3)" << std::endl;
      counter_++;
      return BT::NodeStatus::RUNNING;
    }
    else
    {
      std::cout << "Finished saying hi to " << person_name_ << "!" << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
  }

  void onHalted() override
  {
    std::cout << "SayHi halted." << std::endl;
  }

private:
  std::string person_name_;
  int counter_ = 0;
};

using namespace BT;

BT_REGISTER_ROS_NODES(factory, params)
{

    factory.registerNodeType<SayHi>("SayHi");

    //LOGS
    factory.registerNodeType<DebugLog>("DebugLog");
    factory.registerNodeType<InfoLog>("InfoLog");
    factory.registerNodeType<WarnLog>("WarnLog");
    factory.registerNodeType<ErrorLog>("ErrorLog");
    factory.registerNodeType<FatalLog>("FatalLog");
        
    //TF
    factory.registerNodeType<LookupTransformNode>("LookupTransform");           
    factory.registerNodeType<GetTransformDistanceNode>("GetTransformDistance"); 
    factory.registerNodeType<GetTransformAnglesNode>("GetTransformAngles");
    factory.registerNodeType<GetTransformHorizontalDistanceNode>("GetTransformHorizontalDistance");
    
    //PRIMITIVE SUBSCRIBERS
    factory.registerNodeType<AutoSerSubscriber<std_msgs::msg::Empty>>("MonitorAutoStdEmpty",params);
    factory.registerNodeType<AutoSerSubscriber<std_msgs::msg::Bool>>("MonitorAutoStdBool",params);
    factory.registerNodeType<AutoSerSubscriber<std_msgs::msg::Int8>>("MonitorAutoStdChar",params);
    factory.registerNodeType<AutoSerSubscriber<std_msgs::msg::UInt8>>("MonitorAutoStdUChar",params);
    factory.registerNodeType<AutoSerSubscriber<std_msgs::msg::Int16>>("MonitorAutoStdShort",params);
    factory.registerNodeType<AutoSerSubscriber<std_msgs::msg::UInt16>>("MonitorAutoStdUShort",params);
    factory.registerNodeType<AutoSerSubscriber<std_msgs::msg::Int32>>("MonitorAutoStdInt",params);
    factory.registerNodeType<AutoSerSubscriber<std_msgs::msg::UInt32>>("MonitorAutoStdUInt",params);
    factory.registerNodeType<AutoSerSubscriber<std_msgs::msg::Int64>>("MonitorAutoStdLong",params);
    factory.registerNodeType<AutoSerSubscriber<std_msgs::msg::UInt64>>("MonitorAutoStdULong",params);
    factory.registerNodeType<AutoSerSubscriber<std_msgs::msg::Float32>>("MonitorAutoStdFloat",params);
    factory.registerNodeType<AutoSerSubscriber<std_msgs::msg::Float64>>("MonitorAutoStdDouble",params);
    factory.registerNodeType<AutoSerSubscriber<std_msgs::msg::String>>("MonitorAutoStdString",params);
    factory.registerNodeType<AutoSerSubscriber<geometry_msgs::msg::PoseStamped>>("MonitorAutoPoseStamped",params);
    factory.registerNodeType<JsonSerSubscriber<geometry_msgs::msg::PoseStamped>>("MonitorJsonPoseStamped",params);

    // Possible variant of PRIMITIVE SUBSCRIBER WITH JSON Serialization
    factory.registerNodeType<JsonSerSubscriber<std_msgs::msg::Int32>>("MonitorJsonStdInt",params);
    
    //PRIMITIVE PUBLISHERS
    factory.registerNodeType<AutoDesPublisher<std_msgs::msg::Empty>>("PublishAutoStdEmpty",params);
    factory.registerNodeType<AutoDesPublisher<std_msgs::msg::Bool>>("PublishAutoStdBool",params);
    factory.registerNodeType<AutoDesPublisher<std_msgs::msg::Int8>>("PublishAutoStdChar",params);
    factory.registerNodeType<AutoDesPublisher<std_msgs::msg::UInt8>>("PublishAutoStdUChar",params);
    factory.registerNodeType<AutoDesPublisher<std_msgs::msg::Int16>>("PublishAutoStdShort",params);
    factory.registerNodeType<AutoDesPublisher<std_msgs::msg::UInt16>>("PublishAutoStdUShort",params);
    factory.registerNodeType<AutoDesPublisher<std_msgs::msg::Int32>>("PublishAutoStdInt",params);
    factory.registerNodeType<AutoDesPublisher<std_msgs::msg::UInt32>>("PublishAutoStdUInt",params);
    factory.registerNodeType<AutoDesPublisher<std_msgs::msg::Int64>>("PublishAutoStdLong",params);   
    factory.registerNodeType<AutoDesPublisher<std_msgs::msg::UInt64>>("PublishAutoStdULong",params);
    factory.registerNodeType<AutoDesPublisher<std_msgs::msg::Float32>>("PublishAutoStdFloat",params);
    factory.registerNodeType<AutoDesPublisher<std_msgs::msg::Float64>>("PublishAutoStdDouble",params);
    factory.registerNodeType<AutoDesPublisher<std_msgs::msg::String>>("PublishAutoStdString",params);
    factory.registerNodeType<AutoDesPublisher<geometry_msgs::msg::PoseStamped>>("PublishAutoPoseStamped",params);
    
    //PRIMITIVE SERVICES
    factory.registerNodeType<AutoDesJsonSerServiceClient<std_srvs::srv::Empty>>("ServiceAutoCallJsonEmpty",params);
    factory.registerNodeType<AutoDesJsonSerServiceClient<std_srvs::srv::SetBool>>("ServiceAutoCallJsonBool",params);
    factory.registerNodeType<AutoDesJsonSerServiceClient<std_srvs::srv::Trigger>>("ServiceAutoCallJsonTrigger",params);

    //TEST ACTIONS
    factory.registerNodeType<AutoDesJsonSerActionClient<btcpp_ros2_interfaces::action::Sleep>>("ActionAutoCallJsonSleep",params);
    factory.registerNodeType<AutoDesJsonSerActionClient<btcpp_ros2_interfaces::action::ExecuteTree>>("ActionAutoCallJsonExecuteTree",params);
    
    //TEST
    factory.registerNodeType<AutoDesPublisher<btcpp_ros2_interfaces::msg::NodeStatus>>("PublishNodeStatus",params);
    factory.registerNodeType<AutoDesPublisher<btcpp_ros2_interfaces::msg::CustomMsg>>("PublishCustomMsg",params);
    factory.registerNodeType<AutoDesPublisher<std_msgs::msg::Int8MultiArray>>("PublishStdMultiArray",params);
    factory.registerNodeType<SmartJsonSerSubscriber<btcpp_ros2_interfaces::msg::NodeStatus>>("MonitorSmartJsonNodeStatus",params);
    factory.registerNodeType<SmartJsonSerSubscriber<btcpp_ros2_interfaces::msg::CustomMsg>>("MonitorSmartJsonCustomMsg",params);


};