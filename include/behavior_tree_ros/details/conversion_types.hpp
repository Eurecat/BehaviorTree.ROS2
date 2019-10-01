#ifndef BEHAVIOR_TREE_ROS_CONVERSION_TYPES
#define BEHAVIOR_TREE_ROS_CONVERSION_TYPES

#include <behaviortree_cpp/basic_types.h>

namespace BT
{
    template <>
    inline Any convertFromString<Any>(StringView str)
    {
        return Any(str);
    }

    template <>
    inline nlohmann::json convertFromString<nlohmann::json>(StringView str)
    {
        return nlohmann::json(str);
    }
}

#endif
