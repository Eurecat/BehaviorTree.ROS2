#ifndef BT_ROS_TYPES_CONVERSION
#define BT_ROS_TYPES_CONVERSION

#include <behaviortree_cpp_v3/bt_factory.h>

#include "behavior_tree_ros/3rdparty/nlohmann/json.hpp"

namespace BT
{
    template <>
    inline nlohmann::json convertFromString<nlohmann::json>(StringView str)
    {
        return nlohmann::json::parse(str);
    }

    template <>
    inline std::string toStr<nlohmann::json>(nlohmann::json json)
    {
        return json.dump();
    }
}

#endif
