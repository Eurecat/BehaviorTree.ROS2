#include "BehaviorTreeNode.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behavior_tree_node");
    UPO::BehaviorTreeNode behavior_tree_node {};

    ros::spin();

    return 0;
}

