#ifndef BEHAVIOR_TREE_ROS_CONVERSION_JSON
#define BEHAVIOR_TREE_ROS_CONVERSION_JSON

#include <behaviortree_cpp/basic_types.h>

#include "nlohmann/json.hpp"

namespace BT_ROS
{
    BT::Any json2Any(const nlohmann::json& _json);
}

#endif
