#include "behaviortree_ros2/behaviortree_ros2.hpp"

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

#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

//TEST!
#include <std_msgs/msg/int8_multi_array.hpp>
#include "btcpp_ros2_interfaces/action/sleep.hpp"
#include "btcpp_ros2_interfaces/action/execute_tree.hpp"
#include "btcpp_ros2_interfaces/msg/custom_msg.hpp"



using namespace BT;


BT_REGISTER_ROS_NODES(factory, params)
{
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