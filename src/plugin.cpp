#include <behaviortree_cpp/bt_factory.h>

#include "behavior_tree_ros/GetMessageFieldNode.hpp"
#include "behavior_tree_ros/GetRandomMessageField.hpp"
#include "behavior_tree_ros/FindByFieldValueNode.hpp"
#include "behavior_tree_ros/ForEachLoopNode.hpp"

#include "behavior_tree_ros/SubscriberNode.hpp"


BT_REGISTER_NODES(factory)
{
    using namespace BT_ROS;

    factory.registerNodeType<GetMessageFieldNode>("GetMessageField");
    factory.registerNodeType<GetRandomMessageFieldNode>("GetRandomMessageField");
    factory.registerNodeType<FindByFieldValueNode>("FindByFieldValue");
    factory.registerNodeType<ForEachLoopNode>("ForEachLoop");
}
