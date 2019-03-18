#include <behaviortree_cpp/bt_factory.h>

#include "behavior_tree_ros/actions/GetMessageFieldNode.hpp"
#include "behavior_tree_ros/actions/GetRandomMessageField.hpp"
#include "behavior_tree_ros/actions/FindByFieldValueNode.hpp"
#include "behavior_tree_ros/actions/loggers.hpp"
#include "behavior_tree_ros/decorators/ForEachLoopNode.hpp"

#include "behavior_tree_ros/SubscriberNode.hpp"


BT_REGISTER_NODES(factory)
{
    using namespace BT_ROS;

    factory.registerNodeType<GetMessageFieldNode>("GetMessageField");
    factory.registerNodeType<GetRandomMessageFieldNode>("GetRandomMessageField");
    factory.registerNodeType<FindByFieldValueNode>("FindByFieldValue");
    factory.registerNodeType<ForEachLoopNode<nlohmann::json>>("ForEachLoop");

    factory.registerNodeType<InfoLogger>("InfoLog");
}
