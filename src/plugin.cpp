#include <behaviortree_cpp/bt_factory.h>

#include "behavior_tree_ros/actions/GetMessageFieldNode.hpp"
#include "behavior_tree_ros/actions/GetRandomMessageField.hpp"
#include "behavior_tree_ros/actions/FindByFieldValueNode.hpp"
#include "behavior_tree_ros/actions/ConvertJsonToNode.hpp"
#include "behavior_tree_ros/actions/Logger.hpp"
#include "behavior_tree_ros/actions/GetSizeNode.hpp"
#include "behavior_tree_ros/actions/CopyNode.hpp"
#include "behavior_tree_ros/actions/InitializeNode.hpp"
#include "behavior_tree_ros/decorators/ForEachLoopNode.hpp"
#include "behavior_tree_ros/actions/ConvertMessageFieldNode.hpp"
#include "behavior_tree_ros/actions/ConvertRandomMessageFieldNode.hpp"

#include "behavior_tree_ros/SubscriberNode.hpp"

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
}
