#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/serialized_sub_node.hpp"
#include "behaviortree_ros2/serialized_pub_node.hpp"
#include "behaviortree_ros2/serialized_srv_node.hpp"
#include "behaviortree_ros2/serialized_act_node.hpp"

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

//TEST!
#include <std_msgs/msg/int8_multi_array.hpp>
#include "btcpp_ros2_interfaces/action/sleep.hpp"
#include "btcpp_ros2_interfaces/action/execute_tree.hpp"
#include "btcpp_ros2_interfaces/msg/custom_msg.hpp"



using namespace BT;


BT_REGISTER_ROS_NODES(factory, params)
{
    //PRIMITIVE SUBSCRIBERS
    factory.registerNodeType<SmartSerializedSubscriber<std_msgs::msg::Empty>>("MonitorStdEmpty",params);
    factory.registerNodeType<SmartSerializedSubscriber<std_msgs::msg::Bool>>("MonitorStdBool",params);
    factory.registerNodeType<SmartSerializedSubscriber<std_msgs::msg::Int8>>("MonitorStdChar",params);
    factory.registerNodeType<SmartSerializedSubscriber<std_msgs::msg::UInt8>>("MonitorStdUChar",params);
    factory.registerNodeType<SmartSerializedSubscriber<std_msgs::msg::Int16>>("MonitorStdShort",params);
    factory.registerNodeType<SmartSerializedSubscriber<std_msgs::msg::UInt16>>("MonitorStdUShort",params);
    factory.registerNodeType<SmartSerializedSubscriber<std_msgs::msg::Int32>>("MonitorStdInt",params);
    factory.registerNodeType<SmartSerializedSubscriber<std_msgs::msg::UInt32>>("MonitorStdUInt",params);
    factory.registerNodeType<SmartSerializedSubscriber<std_msgs::msg::Int64>>("MonitorStdLong",params);
    factory.registerNodeType<SmartSerializedSubscriber<std_msgs::msg::UInt64>>("MonitorStdULong",params);
    factory.registerNodeType<SmartSerializedSubscriber<std_msgs::msg::Float32>>("MonitorStdFloat",params);
    factory.registerNodeType<SmartSerializedSubscriber<std_msgs::msg::Float64>>("MonitorStdDouble",params);
    factory.registerNodeType<SmartSerializedSubscriber<std_msgs::msg::String>>("MonitorStdString",params);

    //PRIMITIVE PUBLISHERS
    factory.registerNodeType<AutomaticPublisher<std_msgs::msg::Empty>>("PublishStdEmpty",params);
    factory.registerNodeType<AutomaticPublisher<std_msgs::msg::Bool>>("PublishStdBool",params);
    factory.registerNodeType<AutomaticPublisher<std_msgs::msg::Int8>>("PublishStdChar",params);
    factory.registerNodeType<AutomaticPublisher<std_msgs::msg::UInt8>>("PublishStdUChar",params);
    factory.registerNodeType<AutomaticPublisher<std_msgs::msg::Int16>>("PublishStdShort",params);
    factory.registerNodeType<AutomaticPublisher<std_msgs::msg::UInt16>>("PublishStdUShort",params);
    factory.registerNodeType<AutomaticPublisher<std_msgs::msg::Int32>>("PublishStdInt",params);
    factory.registerNodeType<AutomaticPublisher<std_msgs::msg::UInt32>>("PublishStdUInt",params);
    factory.registerNodeType<AutomaticPublisher<std_msgs::msg::Int64>>("PublishStdLong",params);   
    factory.registerNodeType<AutomaticPublisher<std_msgs::msg::UInt64>>("PublishStdULong",params);
    factory.registerNodeType<AutomaticPublisher<std_msgs::msg::Float32>>("PublishStdFloat",params);
    factory.registerNodeType<AutomaticPublisher<std_msgs::msg::Float64>>("PublishStdDouble",params);
    factory.registerNodeType<AutomaticPublisher<std_msgs::msg::String>>("PublishStdString",params);

    // //PRIMITIVE SERVICES
    factory.registerNodeType<AutomaticServiceClient<std_srvs::srv::Empty>>("CallEmptyService",params);
    factory.registerNodeType<AutomaticServiceClient<std_srvs::srv::SetBool>>("CallSetBoolService",params);
    
    // //TEST ACTIONS
    factory.registerNodeType<AutomaticSimpleActionClient<btcpp_ros2_interfaces::action::Sleep>>("TestActionSleep",params);
    factory.registerNodeType<AutomaticSimpleActionClient<btcpp_ros2_interfaces::action::ExecuteTree>>("TestExecuteTree",params);
    //TEST
    factory.registerNodeType<AutomaticPublisher<btcpp_ros2_interfaces::msg::NodeStatus>>("PublishNodeStatus",params);
    factory.registerNodeType<SmartSerializedSubscriber<btcpp_ros2_interfaces::msg::NodeStatus>>("MonitorNodeStatus",params);
    factory.registerNodeType<AutomaticPublisher<btcpp_ros2_interfaces::msg::CustomMsg>>("PublishCustomMsg",params);
    factory.registerNodeType<SmartSerializedSubscriber<btcpp_ros2_interfaces::msg::CustomMsg>>("MonitorCustomMsg",params);
    factory.registerNodeType<AutomaticPublisher<std_msgs::msg::Int8MultiArray>>("PublishStdMultiArray",params);
};