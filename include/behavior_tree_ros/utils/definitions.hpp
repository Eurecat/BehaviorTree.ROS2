#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <map>

namespace BT_ROS
{
namespace Utils
{
    //maps with enum or enum class as keys have a lot of issues (eg: accesing wrong entries or not compiling).
    //This ensures that a proper hash is computed. A new type is defined for ease of use.
    //Unashamedly stolen from https://stackoverflow.com/a/24847480
    struct EnumClassHash
    {
        template <typename T>
        size_t operator()(T t) const
        {
            return static_cast<size_t>(t);
        }
    };

    template <typename Key>
    using HashType = typename std::conditional<std::is_enum<Key>::value, EnumClassHash, std::hash<Key>>::type;

    template <typename Key, typename T>
    using UnorderedMap = std::unordered_map<Key, T, HashType<Key>>;
}
}

#endif
