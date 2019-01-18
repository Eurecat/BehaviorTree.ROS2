#ifndef BEHAVIOR_TREE_ROS_CONVERSION_TYPES
#define BEHAVIOR_TREE_ROS_CONVERSION_TYPES

#include <behaviortree_cpp/basic_types.h>

namespace BT
{
    //TODO: check numeric range
    template <>
    inline uint8_t convertFromString<uint8_t>(const StringView& str)
    {
        const auto result = std::stoul(str.data());
        return result;
    }

    template <>
    inline uint16_t convertFromString<uint16_t>(const StringView& str)
    {
        const auto result = std::stoul(str.data());
        return result;
    }

    template <>
    inline float convertFromString<float>(const StringView& str)
    {
        const auto result = std::stod(str.data());
        return result;
    }
}

#endif
