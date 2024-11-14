// Copyright 2024 Marq Rasmussen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright
// notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#include <thread>
#pragma warning(pop)
#else
#include <thread>
#endif

#include "tree_wrapper.hpp"
#include "behaviortree_ros2/bt_utils.hpp"

#include "behaviortree_cpp/loggers/groot2_publisher.h"


namespace
{
static const auto kLogger = rclcpp::get_logger("bt_action_server");
}

namespace BT
{

  TreeWrapper::TreeWrapper(const rclcpp::Node::SharedPtr& node)
    : node_(node)
  {
    /*param_listener_ = std::make_shared<bt_server::ParamListener>(node_);
    params_ = param_listener_->get_params();*/
    global_blackboard_ = BT::Blackboard::create();
  }

  TreeWrapper::~TreeWrapper()
  {}

  void TreeWrapper::executeRegistration(int64_t server_port)
  {
    // Before executing check if we have new Behaviors or Subtrees to reload
    factory_.clearRegisteredBehaviorTrees();

    params_ = param_listener_->get_params();
    params_.groot2_port = server_port;
    // user defined method
    registerNodesIntoFactory(factory_);
    // load plugins from multiple directories
    RegisterPlugins(params_, factory_, node_);
    // load trees (XML) from multiple directories
    RegisterBehaviorTrees(params_, factory_, node_);

    factory_initialized_ = true;
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  TreeWrapper::get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

  rclcpp::Node::SharedPtr TreeWrapper::node()
  {
    return node_;
  }

  /*void TreeWrapper::execute(
      const std::shared_ptr<GoalHandleExecuteTree> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    auto action_result = std::make_shared<ExecuteTree::Result>();

    // Before executing check if we have new Behaviors or Subtrees to reload
    if(param_listener->is_old(params))
    {
      executeRegistration();
    }

    // Loop until something happens with ROS or the node completes
    try
    {
      // This blackboard will be owned by "MainTree". It parent is p_->global_blackboard
      auto root_blackboard = BT::Blackboard::create(global_blackboard);

      tree = factory.createTree(goal->target_tree, root_blackboard);
      tree_name = goal->target_tree;
      payload = goal->payload;

      // call user defined function after the tree has been created
      onTreeCreated(tree);
      groot_publisher.reset();
      groot_publisher =
          std::make_shared<BT::Groot2Publisher>(tree, params.groot2_port);

      // Loop until the tree is done or a cancel is requested
      const auto period =
          std::chrono::milliseconds(static_cast<int>(1000.0 / params.tick_frequency));
      auto loop_deadline = std::chrono::steady_clock::now() + period;

      // operations to be done if the tree execution is aborted, either by
      // cancel requested or by onLoopAfterTick()
      auto stop_action = [this, &action_result](BT::NodeStatus status,
                                                const std::string& message) {
        tree.haltTree();
        action_result->node_status = ConvertNodeStatus(status);
        // override the message value if the user defined function returns it
        if(auto msg = onTreeExecutionCompleted(status, true))
        {
          action_result->return_message = msg.value();
        }
        else
        {
          action_result->return_message = message;
        }
        RCLCPP_WARN(kLogger, action_result->return_message.c_str());
      };

      while(rclcpp::ok() && status == BT::NodeStatus::RUNNING)
      {
        if(goal_handle->is_canceling())
        {
          stop_action(status, "Action Server canceling and halting Behavior Tree");
          goal_handle->canceled(action_result);
          return;
        }

        // tick the tree once and publish the action feedback
        status = tree.tickExactlyOnce();

        if(const auto res = onLoopAfterTick(status); res.has_value())
        {
          stop_action(res.value(), "Action Server aborted by onLoopAfterTick()");
          goal_handle->abort(action_result);
          return;
        }

        if(const auto res = onLoopFeedback(); res.has_value())
        {
          auto feedback = std::make_shared<ExecuteTree::Feedback>();
          feedback->message = res.value();
          goal_handle->publish_feedback(feedback);
        }

        const auto now = std::chrono::steady_clock::now();
        if(now < loop_deadline)
        {
          tree.sleep(std::chrono::duration_cast<std::chrono::system_clock::duration>(
              loop_deadline - now));
        }
        loop_deadline += period;
      }
    }
    catch(const std::exception& ex)
    {
      action_result->return_message = std::string("Behavior Tree exception:") + ex.what();
      RCLCPP_ERROR(kLogger, action_result->return_message.c_str());
      goal_handle->abort(action_result);
      return;
    }

    // Call user defined onTreeExecutionCompleted function.
    // Override the message value if the user defined function returns it
    if(auto msg = onTreeExecutionCompleted(status, false))
    {
      action_result->return_message = msg.value();
    }
    else
    {
      action_result->return_message =
          std::string("Tree finished with status: ") + BT::toStr(status);
    }

    // set the node_status result to the action
    action_result->node_status = ConvertNodeStatus(status);

    // return success or aborted for the action result
    if(status == BT::NodeStatus::SUCCESS)
    {
      RCLCPP_INFO(kLogger, action_result->return_message.c_str());
      goal_handle->succeed(action_result);
    }
    else
    {
      RCLCPP_ERROR(kLogger, action_result->return_message.c_str());
      goal_handle->abort(action_result);
    }
  }*/

  bool TreeWrapper::IsTreeLoaded()
  {
    return is_tree_loaded_;
  }

  const std::string& TreeWrapper::treeName() const
  {
    return tree_name_;
  }

  const std::string& TreeWrapper::goalPayload() const
  {
    return payload_;
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

  bool TreeWrapper::HasExecutionTerminated(){return executed_;}

  void TreeWrapper::SetExecuted(bool executed){ executed_ = executed; };
}  // namespace BT
