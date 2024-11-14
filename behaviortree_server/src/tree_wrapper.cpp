#include "tree_wrapper.hpp"

namespace BT
{

  TreeWrapper::TreeWrapper(const rclcpp::Node::SharedPtr& node)
    : node_(node)
  {
    global_blackboard_ = BT::Blackboard::create();
  }

  TreeWrapper::~TreeWrapper()
  {}


  rclcpp::Node::SharedPtr TreeWrapper::node()
  {
    return node_;
  }

  bool TreeWrapper::IsTreeLoaded()
  {
    return is_tree_loaded_;
  }

  const std::string& TreeWrapper::treeName() const
  {
    return tree_name_;
  }

  const BT::Tree& TreeWrapper::tree() const
  {
    return tree_;
  }

  BT::Blackboard::Ptr TreeWrapper::globalBlackboard()
  {
    return global_blackboard_;
  }

  BT::BehaviorTreeFactory& TreeWrapper::factory()
  {
    return factory_;
  }

  bool TreeWrapper::ResetTree()
  {
      if(IsTreeLoaded() )
      {
          tree_.haltTree();
          is_tree_loaded_ = false;
          return true;
      }
      else
          return false;
  }

  void TreeWrapper::RemoveTree()
  {
     ResetTree();
  }

  bool TreeWrapper::HasExecutionTerminated(){ return executed_; }

  void TreeWrapper::SetExecuted(bool executed){ executed_ = executed; };
}  // namespace BT
