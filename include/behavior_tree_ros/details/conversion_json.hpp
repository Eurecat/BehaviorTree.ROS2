#ifndef BEHAVIOR_TREE_ROS_CONVERSION_JSON
#define BEHAVIOR_TREE_ROS_CONVERSION_JSON

#include <behaviortree_cpp/basic_types.h>

//TODO: do not expose this
#include "behavior_tree_ros/3rdparty/nlohmann/json.hpp"

namespace BT_ROS
{
    BT::Any json2Any(const nlohmann::json& _json);
    bool areJsonAndAnyEquals(const nlohmann::json& _json, const BT::Any& _any);
}

#endif
