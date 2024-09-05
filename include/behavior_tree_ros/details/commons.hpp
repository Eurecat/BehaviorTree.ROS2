#ifndef TREE_ROS_COMMONS_HPP
#define TREE_ROS_COMMONS_HPP

#include <string>

#include <behaviortree_cpp_v3/bt_factory.h>

namespace BT_ROS
{
    void InitializeBlackboard(const std::string& _tree_file, BT::Blackboard::Ptr blackboard_ptr, const bool sync_bb = false);
}
#endif