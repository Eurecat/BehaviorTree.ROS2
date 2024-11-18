#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/bt_factory.h"

#include "behaviortree_ros2/bt_utils.hpp"

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

// generated file
#include "bt_executor_parameters.hpp"

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

    ~TreeWrapper();

    /// @brief Gets the rclcpp::Node pointer
    rclcpp::Node::SharedPtr node() { return node_; }

    /// @brief Tree being executed.
    const BT::Tree& tree() const { return tree_; }

    /// @brief Pointer to the global blackboard
    BT::Blackboard::Ptr globalBlackboard() { return global_blackboard_; }

    /// @brief Pointer to the global blackboard
    BT::BehaviorTreeFactory& factory() { return factory_; }

    bool IsTreeLoaded() { return is_tree_loaded_; }

    bool HasExecutionTerminated() { return executed_; }

    bool ResetTree();

    void RemoveTree();

    void SetExecuted(bool executed) { executed_ = executed; }

    bool executed_{false};

    bool is_tree_loaded_{false};

    void LoadAllPlugins();
    void LoadPluginsFromROS(std::vector<std::string> ros_plugins_folders);
    void LoadPluginsFromFolder();

    void InitializeBlackboard();
    void InitializeBlackboardFile(const std::string& abs_file_path, const bool sync_bb);

    void InitGrootPublisher();

    void CreateTree(const std::string& full_path);

    BT::Tree tree_;

    BT::BehaviorTreeFactory factory_;

    std::shared_ptr<BT::Groot2Publisher> groot_publisher_;

    BT::Blackboard::Ptr global_blackboard_;
    bool factory_initialized_ = false;

    rclcpp::Node::SharedPtr node_;
    
    //Tree Status
    std::string execution_tree_status_ {};
    std::string execution_tree_error_ {};
    BT::NodeStatus status_ { BT::NodeStatus::IDLE };

    //parameters
    bt_server::Params params_;
    std::vector<std::string> ros_plugin_directories_;
    int tree_server_port_;
    int tree_publisher_port_;

    std::vector<std::string> tree_bb_init_ {};
  };

}  // namespace BT
