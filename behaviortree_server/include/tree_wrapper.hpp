#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_file_logger.h>
#include <behaviortree_cpp/loggers/bt_minitrace_logger.h>
#include <behaviortree_cpp/loggers/bt_zmq_publisher.h>

#include "behaviortree_ros2/bt_utils.hpp"

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include "behaviortree_server_interfaces/srv/get_tree_status.hpp"
#include "behaviortree_server_interfaces/msg/transition.hpp"
#include "behaviortree_server_interfaces/msg/bb_entry.hpp"

// generated file
#include "bt_executor_parameters.hpp"

using TreeStatus = behaviortree_server_interfaces::msg::TreeExecutionStatus;
using Transition = behaviortree_server_interfaces::msg::Transition;
using BBEntry = behaviortree_server_interfaces::msg::BBEntry;

namespace BT
{

  /**
   * @brief TreeWrapper class hosts a BT
   */
  class TreeWrapper
  {

  public:

    /**
     * @brief Constructor to use when an already existing node should be used.
    */
    TreeWrapper(const rclcpp::Node::SharedPtr& node);

    /// @brief Destructor
    ~TreeWrapper();

    /// @brief Gets the rclcpp::Node pointer
    rclcpp::Node::SharedPtr node() { return node_; }

    /// @brief Tree being executed.
    const BT::Tree& tree() const { return tree_; }

    /// @brief Pointer to the global blackboard
    BT::Blackboard::Ptr globalBlackboard() { return global_blackboard_; }

    /// @brief Pointer to the global blackboard
    BT::BehaviorTreeFactory& factory() { return factory_; }

    /// @brief Call Init Status Publisher
    void InitializeStatusPublisher();

    /// @brief Call Load All plugins
    void LoadAllPlugins();

    /// @brief Call Init Blackboard
    void InitializeBlackboard();

    /// @brief Create Tree Status message
    TreeStatus buildTreeExecutionStatus();

    /// @brief Publish Execution Status with new status
    void UpdatePublishTreeExecutionStatus(const BT::NodeStatus status, const bool avoid_duplicate = true);

    /// @brief Publish Execution Status
    void PublishExecutionStatus(bool error=false, std::string error_data="");

    /// @brief Check if tree is loaded
    bool IsTreeLoaded() { return is_tree_loaded_; }

    /// @brief Check if tree is loaded
    void SetTreeLoaded(bool loaded) { is_tree_loaded_ = loaded; }

    /// @brief Destroy the tree
    void RemoveTree();

    /// @brief Reset the tree
    bool ResetTree();

    /// @brief Create the tree
    void CreateTree(const std::string& full_path);

    /// @brief Init BT Loggers
    void InitializeLoggers();

    /// @brief Get Single Blackboard Update
    void SyncBlackboardUpdateCallback(const BBEntry& _single_upd);

    /// @brief Check if loggers are init
    bool AreLoggersInitialized() { return loggers_init_; }

    /// @brief Check if execution is finished
    bool HasExecutionTerminated() { return executed_; }

    /// @brief Set execution status
    void SetExecuted(bool executed) { executed_ = executed; }

    /// @brief Get Number of Tree nodes
    size_t TreeNodesCount();

    /// @brief  Check if the tree is Paused
    // TODO: Check if tree is Paused
    // - Missing TREE->ISPAUSED IMPLEMENTATION
    //bool IsTreePaused() { return IsTreeLoaded() && tree_->isPaused(); }


    void SyncBlackboardUpdateCallback(const std::vector<BBEntry>& _bulk_upd);

    BT::NodeStatus GetTreeStatus()
    {
        std::lock_guard<std::mutex> lk(status_lock_);
        return status_;
    }

    BT::Tree tree_;

    //Tree Status
    std::string execution_tree_status_ {};
    std::string execution_tree_error_ {};

    //parameters
    bt_server::Params params_;
    std::vector<std::string> ros_plugin_directories_;
    std::string log_folder_;
    bool enable_cout_log_;
    bool enable_minitrace_log_;
    bool enable_rostopic_log_;
    bool enable_file_log_;
    bool enable_zmq_log_;
    int tree_server_port_;
    int tree_publisher_port_;
    int tree_uid_;
    std::string tree_filename_;
    std::string tree_name_;
    std::vector<std::string> tree_bb_init_ {};
    rclcpp::Time start_execution_time_;
    std::set<std::string> loaded_plugins_;

  private:
    void LoadPluginsFromROS(std::vector<std::string> ros_plugins_folders);
    void LoadPluginsFromFolder();

    void InitializeBlackboardFile(const std::string& abs_file_path, const bool sync_bb);
    void InitGrootV2Publisher();
    void ResetLoggers();

    rclcpp::Node::SharedPtr node_;

    BT::BehaviorTreeFactory factory_;
    BT::Blackboard::Ptr global_blackboard_;

    bool executed_{false};
    bool is_tree_loaded_{false};
    bool factory_initialized_ = false;

    //Loggers
    std::unique_ptr<BT::StdCoutLogger>   bt_logger_cout_;
    std::unique_ptr<BT::FileLogger>      bt_logger_file_;
    std::unique_ptr<BT::MinitraceLogger> bt_logger_trace_;
    std::unique_ptr<BT::PublisherZMQ>    bt_logger_zmq_;
    std::shared_ptr<BT::Groot2Publisher> groot_publisher_;

    //Publishers
    rclcpp::Publisher<Transition>::SharedPtr bt_transition_publisher_;
    rclcpp::Publisher<TreeStatus>::SharedPtr bt_execution_status_publisher_;

    //Tree Status
    BT::NodeStatus status_ { BT::NodeStatus::IDLE };

    std::atomic_bool loggers_init_{false};

    std::mutex status_lock_;

  };

}  // namespace BT
