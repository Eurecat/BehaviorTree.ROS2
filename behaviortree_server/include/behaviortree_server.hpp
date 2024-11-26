#ifndef BEHAVIORTREE_SERVER_HPP
#define BEHAVIORTREE_SERVER_HPP

#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/empty.hpp"
#include "behaviortree_server_interfaces/msg/bb_entry.hpp"
#include "behaviortree_server_interfaces/msg/tree_execution_status.hpp"

#include "behaviortree_server_interfaces/srv/load_tree.hpp"
#include "behaviortree_server_interfaces/srv/get_tree_status_by_id.hpp"
#include "behaviortree_server_interfaces/srv/get_tree_status.hpp"
#include "behaviortree_server_interfaces/srv/get_bb_values.hpp"
#include "behaviortree_server_interfaces/srv/tree_request.hpp"
#include "behaviortree_server_interfaces/srv/get_all_trees_status.hpp"

#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/eut/eut_debug.h"
#include <behaviortree_cpp/bt_factory.h>

#include "ros2_launch_manager.hpp"
#include "utils.hpp"
#include "yaml-cpp/yaml.h"

#include <chrono>

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

using BBEntry = behaviortree_server_interfaces::msg::BBEntry;
using TreeStatus = behaviortree_server_interfaces::msg::TreeExecutionStatus;
using LoadTreeSrv = behaviortree_server_interfaces::srv::LoadTree;
using TreeRequestSrv = behaviortree_server_interfaces::srv::TreeRequest;
using GetBBValuesSrv = behaviortree_server_interfaces::srv::GetBBValues;
using GetTreeStatusSrv = behaviortree_server_interfaces::srv::GetTreeStatusByID;
using GetAllTreeStatusSrv = behaviortree_server_interfaces::srv::GetAllTreesStatus;
using EmptySrv = std_srvs::srv::Empty;

namespace BT_SERVER
{
  class TreeProcessInfo
  {
    public:
      TreeProcessInfo(std::string in_tree_name, pid_t in_pid)
      {
          tree_name = in_tree_name;
          pid = in_pid;
      };
      pid_t pid;
      std::string tree_name;
      TreeStatus tree_status;
      rclcpp::Subscription<TreeStatus>::SharedPtr status_subscriber;
  };
  class BehaviorTreeServer
  {
    public:
      BehaviorTreeServer(const rclcpp::Node::SharedPtr& node) ;
      ~BehaviorTreeServer();

    private:
      void sync_bb_callback(const BBEntry::SharedPtr msg) const;
      bool LoadTree(const std::shared_ptr<LoadTreeSrv::Request> req, std::shared_ptr<LoadTreeSrv::Response> res);
      bool RosServiceStopCall (std::string tree_name);
      bool RosServiceRestartCall (std::string tree_name);
      bool StopTree(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res);
      bool KillTree(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res);
      bool KillAllTrees(const std::shared_ptr<EmptySrv::Request> req, std::shared_ptr<EmptySrv::Response> res);
      bool RestartTree(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res);
      bool GetSyncBBValues (const std::shared_ptr<GetBBValuesSrv::Request> req, std::shared_ptr<GetBBValuesSrv::Response> res);
      bool GetTreeStatus(const std::shared_ptr<GetTreeStatusSrv::Request> req, std::shared_ptr<GetTreeStatusSrv::Response> res);
      bool GetAllTreeStatus(const std::shared_ptr<GetAllTreeStatusSrv::Request> req, std::shared_ptr<GetAllTreeStatusSrv::Response> res);
      void TreeStatusTopicCB(const TreeStatus::SharedPtr msg);
      void InitializeBlackboard(const std::string& abs_file_path, BT::Blackboard::Ptr blackboard_ptr, const bool sync_bb = false);
      
      //Services
      rclcpp::Service<LoadTreeSrv>::SharedPtr load_tree_srv_;
      rclcpp::Service<TreeRequestSrv>::SharedPtr stop_tree_srv_;
      rclcpp::Service<TreeRequestSrv>::SharedPtr kill_tree_srv_;
      rclcpp::Service<EmptySrv>::SharedPtr kill_all_trees_srv_;
      rclcpp::Service<TreeRequestSrv>::SharedPtr restart_tree_srv_; 
      rclcpp::Service<GetBBValuesSrv>::SharedPtr get_sync_bb_values_srv_; 
      rclcpp::Service<GetTreeStatusSrv>::SharedPtr get_tree_status_srv_;
      rclcpp::Service<GetAllTreeStatusSrv>::SharedPtr get_all_trees_status_srv_;

      //Subscribers
      rclcpp::Subscription<BBEntry>::SharedPtr sync_bb_sub_;
      
      //Publishers
      rclcpp::Publisher<BBEntry>::SharedPtr sync_bb_pub_;

      //Sync BB
      BT::Blackboard::Ptr sync_blackboard_ptr_;

      //BT_FACTORY
      BT::BehaviorTreeFactory bt_factory_;

      //Save Spawned Trees information
      std::map <unsigned int, TreeProcessInfo> uids_to_tree_info;

      //Manage spawn Process using ROS LAUNCH command
      ROS2LaunchManager ros2_launch_manager;

      //Manage Trees_UIDs
      unsigned int trees_UID = 0;

      rclcpp::Node::SharedPtr node_ ;
  };
}
#endif