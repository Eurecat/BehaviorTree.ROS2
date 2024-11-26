#ifndef BEHAVIORTREE_NODE_H
#define BEHAVIORTREE_NODE_H

#include "rclcpp/rclcpp.hpp"

#include "utils.hpp"
#include "tree_wrapper.hpp"

#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "behaviortree_server_interfaces/msg/bb_entry.hpp"
#include "behaviortree_server_interfaces/srv/get_loaded_plugins.hpp"
#include "behaviortree_server_interfaces/srv/get_bb_values.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using BBEntry = behaviortree_server_interfaces::msg::BBEntry;
using GetLoadedPluginsSrv = behaviortree_server_interfaces::srv::GetLoadedPlugins;
using GetTreeStatusSrv = behaviortree_server_interfaces::srv::GetTreeStatus;
using GetBBValues = behaviortree_server_interfaces::srv::GetBBValues;
using TriggerSrv = std_srvs::srv::Trigger;
using EmptySrv = std_srvs::srv::Empty;

namespace BT_SERVER
{
  class BehaviorTreeNode
  {
    public:
      BehaviorTreeNode(const rclcpp::Node::SharedPtr& node);
      ~BehaviorTreeNode() = default;

      void Loop();

      uint loop_rate_ = 30;

    private:

      //TODO:
      //void sendBlackboardUpdates(const BT::Blackboard::SerializedEntriesMap& entries_map);

      void getBlackboardUpdates(const bool just_empty_values = false);
      void SyncBlackboardUpdateCallback(const BBEntry::SharedPtr _topic_msg);

      //TODO:
      // - Missing BLACKBOARD->GETSYNCKEYS
      //std::unordered_set<std::string> getSyncKeys(const bool just_empty_values){return tree_wrapper_.globalBlackboard()->getSyncKeys(just_empty_values);};
      bool LoadTree();
      void getParameters (rclcpp::Node::SharedPtr nh);

      bool GetLoadedPluginsServiceCallback(const std::shared_ptr<GetLoadedPluginsSrv::Request> _request, std::shared_ptr<GetLoadedPluginsSrv::Response> _response);
      bool StopTreeCallback(const std::shared_ptr<EmptySrv::Request> _request, std::shared_ptr<EmptySrv::Response> _response);
      bool PauseTreeCallback(const std::shared_ptr<TriggerSrv::Request> _request, std::shared_ptr<TriggerSrv::Response> _response);
      bool ResumeTreeCallback(const std::shared_ptr<TriggerSrv::Request> _request, std::shared_ptr<TriggerSrv::Response> _response);
      bool RestartTreeCallback(const std::shared_ptr<EmptySrv::Request> _request, std::shared_ptr<EmptySrv::Response> _response);
      bool StatusTreeCallback(const std::shared_ptr<GetTreeStatusSrv::Request> _request, std::shared_ptr<GetTreeStatusSrv::Response> _response);
      void CheckPausedCallback();

      void RemoveTree();
      bool StopTree();

      rclcpp::Node::SharedPtr node_ ;
      BT::TreeWrapper tree_wrapper_;
      std::string trees_folder_;
      std::string tree_name_;

      bool tree_debug_;
      bool tree_auto_restart_;

      //Services
      rclcpp::Service<GetLoadedPluginsSrv>::SharedPtr get_loaded_plugins_srv_;
      rclcpp::Service<EmptySrv>::SharedPtr stop_tree_srv_;
      rclcpp::Service<TriggerSrv>::SharedPtr pause_tree_srv_;
      rclcpp::Service<TriggerSrv>::SharedPtr resume_tree_srv_;
      rclcpp::Service<EmptySrv>::SharedPtr restart_tree_srv_;
      rclcpp::Service<GetTreeStatusSrv>::SharedPtr get_tree_status_srv_;

      //Subscribers
      rclcpp::Subscription<BBEntry>::SharedPtr sync_bb_sub_;

      //Publishers
      rclcpp::Publisher<BBEntry>::SharedPtr sync_bb_pub_;

      //rclcpp::WallTimer<rclcpp::VoidCallbackType>::SharedPtr check_paused_timer_;
      rclcpp::TimerBase::SharedPtr check_paused_timer_; 
  };
}


#endif  // BEHAVIORTREE_NODE_H