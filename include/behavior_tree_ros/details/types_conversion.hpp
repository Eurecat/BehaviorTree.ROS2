#ifndef BT_ROS_TYPES_CONVERSION
#define BT_ROS_TYPES_CONVERSION

#include <behaviortree_cpp/bt_factory.h>

#include "behavior_tree_ros/3rdparty/nlohmann/json.hpp"

namespace BT
{
    template <>
    inline nlohmann::json convertFromString<nlohmann::json>(StringView str)
    {
        return nlohmann::json(str);
    }
}

#endif
