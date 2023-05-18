#ifndef TRANSFORM_UTILS_HPP
#define TRANSFORM_UTILS_HPP

#include "behaviortree_cpp_v3/basic_types.h"
#include <tf/transform_datatypes.h>

namespace BT
{
    template <>
    std::string toStr<tf::StampedTransform>(tf::StampedTransform transform)
    {
        return std::string("translation:{")
            + std::to_string(transform.getOrigin().getX()) + std::string(",")
            + std::to_string(transform.getOrigin().getY()) + std::string(",")
            + std::to_string(transform.getOrigin().getZ()) 
            + std::string("}, rotation:{")
            + std::to_string(transform.getRotation().getX()) + std::string(",")
            + std::to_string(transform.getRotation().getY()) + std::string(",")
            + std::to_string(transform.getRotation().getZ()) + std::string(",")
            + std::to_string(transform.getRotation().getW())
            + std::string("}");
    }

};

#endif
