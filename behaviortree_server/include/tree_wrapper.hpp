#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

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
    rclcpp::Node::SharedPtr node();

    /// @brief Name of the tree being executed
    const std::string& treeName() const;

    /// @brief Tree being executed.
    const BT::Tree& tree() const;  

    /// @brief Pointer to the global blackboard
    BT::Blackboard::Ptr globalBlackboard();

    /// @brief Pointer to the global blackboard
    BT::BehaviorTreeFactory& factory();

    bool IsTreeLoaded();

    bool HasExecutionTerminated();

    bool ResetTree();

    void RemoveTree();

    void SetExecuted(bool executed);

    bool executed_{false};

    bool is_tree_loaded_{false};

    bt_server::Params params_;

    BT::BehaviorTreeFactory factory_;

    std::shared_ptr<BT::Groot2Publisher> groot_publisher_;

    std::string tree_name_;
    BT::Tree tree_;
    BT::Blackboard::Ptr global_blackboard_;
    bool factory_initialized_ = false;

    rclcpp::Node::SharedPtr node_;
    
    //Tree Status
    std::string execution_tree_status_ {};
    std::string execution_tree_error_ {};
    BT::NodeStatus status_ { BT::NodeStatus::IDLE };

  };

}  // namespace BT
