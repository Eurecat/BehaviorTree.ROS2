#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <btcpp_ros2_interfaces/action/execute_tree.hpp>

class TreeExecutionClient : public rclcpp::Node
{
public:
    using ExecuteTree = btcpp_ros2_interfaces::action::ExecuteTree;
    using GoalHandleExecuteTree = rclcpp_action::ClientGoalHandle<ExecuteTree>;

    explicit TreeExecutionClient(const std::string& target_tree, const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("tree_execution_client", options)
    {
        action_client_ = rclcpp_action::create_client<ExecuteTree>(this, "/bt_action_server_example");

        if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        if (target_tree.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Target tree name must be specified");
            rclcpp::shutdown();
            return;
        }

        send_goal(target_tree);
    }

private:
    rclcpp_action::Client<ExecuteTree>::SharedPtr action_client_;

    void send_goal(const std::string &target_tree)
    {
        auto goal_msg = ExecuteTree::Goal();
        goal_msg.target_tree = target_tree;
        goal_msg.payload = ""; // Optional payload can be set here

        RCLCPP_INFO(this->get_logger(), "Sending goal to execute tree: %s", target_tree.c_str());

        auto send_goal_options = rclcpp_action::Client<ExecuteTree>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            [this](GoalHandleExecuteTree::SharedPtr goal_handle) {
                if (!goal_handle)
                {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by the server, waiting for result");
                }
            };

        send_goal_options.feedback_callback =
            [this](GoalHandleExecuteTree::SharedPtr, const std::shared_ptr<const ExecuteTree::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "Feedback received: %s", feedback->message.c_str());
            };

        send_goal_options.result_callback =
            [this](const GoalHandleExecuteTree::WrappedResult &result) {
                switch (result.code)
                {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Result received: Status=%d, Message=%s",
                                result.result->node_status, result.result->return_message.c_str());
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "Goal was canceled");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    break;
                }
                rclcpp::shutdown();
            };
            
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }
};

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
