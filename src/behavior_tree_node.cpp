#include "BehaviorTreeNode.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behavior_tree_node");
    ros::AsyncSpinner async_spinner { 0 };
    async_spinner.start();

    UPO::BehaviorTreeNode behavior_tree_node {};

    while(ros::ok())
    {
        behavior_tree_node.Loop();
    }

    return 0;
}

