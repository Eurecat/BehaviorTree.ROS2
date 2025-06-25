#ifndef TREE_EXECUTION_CLIENT_HPP
#define TREE_EXECUTION_CLIENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <btcpp_ros2_interfaces/action/execute_tree.hpp>

class TreeExecutionClient : public rclcpp::Node
{
public:
    using ExecuteTree = btcpp_ros2_interfaces::action::ExecuteTree;
    using GoalHandleExecuteTree = rclcpp_action::ClientGoalHandle<ExecuteTree>;

    explicit TreeExecutionClient(const std::string &target_tree, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    rclcpp_action::Client<ExecuteTree>::SharedPtr action_client_;

    void send_goal(const std::string &target_tree);
};

#endif // TREE_EXECUTION_CLIENT_HPP