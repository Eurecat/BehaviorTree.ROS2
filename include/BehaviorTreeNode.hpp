#ifndef BEHAVIOR_TREE_NODE_HPP
#define BEHAVIOR_TREE_NODE_HPP

#include <set>
#include <string>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <actionlib/server/simple_action_server.h>

#include <behavior_tree_ros/GetLoadedPlugins.h>
#include <behavior_tree_ros/LoadTree.h>
#include <behavior_tree_ros/BehaviorTreeAction.h>
#include <behavior_tree_ros/TreeStatus.h>
#include <behavior_tree_ros/GetTreeStatus.h>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/xml_parsing.h>

#include "behavior_tree_ros/details/TreeWrapper.hpp"

namespace BT_ROS
{
    class BehaviorTreeNode final
    {
        private:
            using PluginsService  = behavior_tree_ros::GetLoadedPlugins;
            using LoadTreeService = behavior_tree_ros::LoadTree;
            using StatusService  = behavior_tree_ros::GetTreeStatus;

        public:
            BehaviorTreeNode();
            ~BehaviorTreeNode() = default;

            void Loop();

        private:
            bool GetLoadedPluginsService(PluginsService::Request& _request, PluginsService::Response& _response);
            bool LoadTree(LoadTreeService::Request& _request, LoadTreeService::Response& _response);
            bool StopTree(std_srvs::Empty::Request& _request, std_srvs::Empty::Response& _response);
            bool StatusTree(StatusService::Request& _request, StatusService::Response& _response);
            void LoadAllPlugins();
            void LoadPluginsFromROS();
            void LoadPluginsFromFolder(const std::string& _plugins_folder);
            void LoadPlugin(const std::string& _plugin_path);

            void RemoveTree();

            std::string GetFullPath(const std::string& _file) const;

            void PublishExecutionStatus();

            // Behavior Tree action server callbacks
            void ActionGoalCB();
            void ActionPreemptCB();

        private:
            ros::NodeHandle private_node_handle_ { "~" };
            ros::NodeHandle public_node_handle_;
            ros::Rate loop_rate_;

            ros::ServiceServer get_loaded_plugins_srv_;
            ros::ServiceServer get_tree_status_srv_;
            ros::ServiceServer load_tree_srv_;
            ros::ServiceServer stop_tree_srv_;

            ros::Publisher bt_execution_status_publisher_;

            // Behavior Tree action server
            actionlib::SimpleActionServer<behavior_tree_ros::BehaviorTreeAction> bt_action_server_;

            TreeWrapper service_tree_{"service"};
            std::vector<TreeWrapper*> action_trees_ ;
            //TreeWrapper action_tree_{"action"};
            BT::BehaviorTreeFactory bt_factory_;

            std::set<std::string> loaded_plugins_;
            std::string trees_folder_;
            std::string log_folder_;
            bool enable_cout_log_;
            bool enable_minitrace_log_;
            bool enable_rostopic_log_;
            bool enable_file_log_;
            bool enable_zmq_log_;

            // Execution status report.
            BT::NodeStatus status_ { BT::NodeStatus::IDLE };
            std::string current_tree_ {};

            // Action feedback and status
            behavior_tree_ros::BehaviorTreeFeedback action_feedback_;
            behavior_tree_ros::BehaviorTreeResult action_result_;
    };
}

#endif
