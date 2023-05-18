#include <std_msgs/Int32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <behaviortree_cpp_v3/bt_factory.h>

#include "behavior_tree_ros/details/types_conversion.hpp"

#include "behavior_tree_ros/actions/GetMessageFieldNode.hpp"
#include "behavior_tree_ros/actions/GetRandomMessageField.hpp"
#include "behavior_tree_ros/actions/FindByFieldValueNode.hpp"
#include "behavior_tree_ros/actions/ConvertJsonToNode.hpp"
#include "behavior_tree_ros/actions/Logger.hpp"
#include "behavior_tree_ros/actions/GetSizeNode.hpp"
#include "behavior_tree_ros/actions/CopyNode.hpp"
#include "behavior_tree_ros/actions/InitializeNode.hpp"
#include "behavior_tree_ros/actions/AddKeyValueToJson.hpp"
#include "behavior_tree_ros/actions/AddArrayToJson.hpp"
#include "behavior_tree_ros/actions/SplitStringToJsonArray.hpp"
#include "behavior_tree_ros/decorators/ForEachLoopNode.hpp"
#include "behavior_tree_ros/actions/ConvertMessageFieldNode.hpp"
#include "behavior_tree_ros/actions/ConvertRandomMessageFieldNode.hpp"
#include "behavior_tree_ros/actions/LookupTransformNode.hpp"
#include "behavior_tree_ros/actions/GetTransformDistanceNode.hpp"
#include "behavior_tree_ros/actions/GetTransformHorizontalDistanceNode.hpp"
#include "behavior_tree_ros/actions/GetTransformOriginNode.hpp"
#include "behavior_tree_ros/actions/GetTransformAnglesNode.hpp"
#include "behavior_tree_ros/actions/LoadYamlFileNode.hpp"

#include "behavior_tree_ros/actions/TfStampedTransformUtils.hpp"

#include "behavior_tree_ros/PublisherNode.hpp"
#include "behavior_tree_ros/SubscriberNode.hpp"
#include "behavior_tree_ros/ServiceClientNode.hpp"
#include "behavior_tree_ros/SimpleActionClientNode.hpp"

#include <behavior_tree_ros/BehaviorTreeAction.h>
#include <behavior_tree_ros/HandShakeAction.h> 
#include <behavior_tree_ros/ExchangeInfoAction.h>

namespace BT_ROS
{
    std::string json2String(const nlohmann::json& _input)
    {
        return _input.dump();
    }

    double json2Double(const nlohmann::json& _input)
    {
        return atof( _input.dump().c_str() );
    }
}

BT_REGISTER_NODES(factory)
{
    using namespace BT_ROS;

    factory.registerNodeType<GetMessageFieldNode>("GetMessageField");
    factory.registerNodeType<ConvertJsonToNode<std::string>>("ConvertJsonToString");
    factory.registerNodeType<ConvertJsonToNode<double>>("ConvertJsonToDouble");
    factory.registerNodeType<ConvertJsonToNode<int64_t>>("ConvertJsonToInt64");
    factory.registerNodeType<ConvertJsonToNode<uint64_t>>("ConvertJsonToUint64");
    factory.registerNodeType<GetRandomMessageFieldNode>("GetRandomMessageField");
    factory.registerNodeType<FindByFieldValueNode>("FindByFieldValue");
    factory.registerNodeType<ForEachLoopNode<nlohmann::json>>("ForEachLoop");
    factory.registerNodeType<GetSizeNode<nlohmann::json>>("GetJsonSize");
    factory.registerNodeType<CopyNode<nlohmann::json>>("CopyJson");
    factory.registerNodeType<InitializeNode<nlohmann::json>>("InitializeJson");
    factory.registerNodeType<ConvertMessageFieldNode>("ConvertMessageField");
    factory.registerNodeType<ConvertRandomMessageFieldNode>("ConvertRandomMessageField");

    factory.registerNodeType<DebugLog>("DebugLog");
    factory.registerNodeType<InfoLog>("InfoLog");
    factory.registerNodeType<WarnLog>("WarnLog");
    factory.registerNodeType<ErrorLog>("ErrorLog");
    factory.registerNodeType<FatalLog>("FatalLog");

    factory.registerNodeType<AddKeyValueToJson>("AddKeyValueToJson");
    factory.registerNodeType<AddArrayToJson>("AddArrayToJson");
    factory.registerNodeType<SplitStringToJsonArray>("SplitStringToJsonArray");

    factory.registerNodeType<LookupTransformNode>("LookupTransform");
    factory.registerNodeType<GetTransformDistanceNode>("GetTransformDistance");
    factory.registerNodeType<GetTransformHorizontalDistanceNode>("GetTransformHorizontalDistance");
    factory.registerNodeType<GetTransformOriginNode>("GetTransformOrigin");
    factory.registerNodeType<GetTransformAnglesNode>("GetTransformAngles");
    factory.registerNodeType<LoadYamlFileNode>("LoadYamlFile");

    factory.registerNodeType<SerializedSubscriber<std_msgs::Int32>>("MonitorStdInt32");
    factory.registerNodeType<SerializedSubscriber<std_msgs::UInt64>>("MonitorStdUInt64");
    factory.registerNodeType<SerializedSubscriber<std_msgs::Float64>>("MonitorStdFloat64");
    factory.registerNodeType<SerializedSubscriber<std_msgs::Bool>>("MonitorStdBool");
    factory.registerNodeType<SerializedSubscriber<std_msgs::String>>("MonitorStdString");

    factory.registerNodeType<AutomaticPublisher<std_msgs::String>>("PublishStdString");
    factory.registerNodeType<AutomaticPublisher<std_msgs::Empty>>("PublishStdEmpty");
    factory.registerNodeType<AutomaticPublisher<std_msgs::Bool>>("PublishStdBool");
    factory.registerNodeType<AutomaticPublisher<std_msgs::Int32>>("PublishStdInt32");
    factory.registerNodeType<AutomaticPublisher<std_msgs::UInt64>>("PublishStdUInt64");
    factory.registerNodeType<AutomaticPublisher<std_msgs::Float64>>("PublishStdFloat64");

    factory.registerNodeType<AutomaticServiceClient<std_srvs::Empty>>("CallEmptyService");
    factory.registerNodeType<AutomaticServiceClient<std_srvs::SetBool>>("CallSetBoolService");
    factory.registerNodeType<AutomaticServiceClient<std_srvs::Trigger>>("CallTriggerService");

    // factory.registerTypeConverter<std::string, nlohmann::json>(BT::convertFromString<nlohmann::json>);
    // factory.registerTypeConverter<nlohmann::json, std::string>(json2String);
    // factory.registerTypeConverter<nlohmann::json, double>(json2Double);

    factory.registerNodeType<SimpleActionClientNode<behavior_tree_ros::BehaviorTreeAction,
                                                    AutomaticDeserialization,
                                                    EmptySerialization,
                                                    EmptySerialization>>("ExecuteRemoteTree");
    factory.registerNodeType<AutomaticSimpleActionClient<behavior_tree_ros::HandShakeAction>>("BTCommandHandShakeAction");
    factory.registerNodeType<AutomaticSimpleActionClient<behavior_tree_ros::ExchangeInfoAction>>("BTCommandExchangeInfoAction");
}
