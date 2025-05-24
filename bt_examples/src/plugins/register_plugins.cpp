#include "behaviortree_ros2/plugins.hpp"
#include "sleep.hpp"

using namespace BT;

BT_REGISTER_ROS_NODES(factory, params)
{
    
    factory.registerNodeType<SleepAction>("SleepAction", params);

};