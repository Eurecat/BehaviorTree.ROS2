#include "behaviortree_ros2/tools/client/tree_execution_client.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("tree_execution_client"), "Usage: ros2 run package_name node_name <target_tree_name>");
        return 1;
    }
    
    std::string target_tree = argv[1];
    auto node = std::make_shared<TreeExecutionClient>(target_tree);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
