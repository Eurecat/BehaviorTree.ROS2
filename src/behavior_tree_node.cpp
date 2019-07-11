#include "BehaviorTreeNode.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behavior_tree_node");
    ros::AsyncSpinner spinner(0);

    UPO::BehaviorTreeNode behavior_tree_node {};

    spinner.start();

    while(ros::ok())
    {
        behavior_tree_node.Loop();
    }

    return 0;
}

