#ifndef BEHAVIOR_TREE_NODE_HPP
#define BEHAVIOR_TREE_NODE_HPP

#include <set>
#include <string>

#include <ros/ros.h>
#include <behavior_tree_ros/GetLoadedPlugins.h>

#include <behavior_tree_core/bt_factory.h>
#include <behavior_tree_logger/bt_cout_logger.h>
#include <behavior_tree_logger/bt_file_logger.h>
#include <behavior_tree_logger/bt_minitrace_logger.h>
#include <behavior_tree_logger/bt_zmq_publisher.h>

#include <Blackboard/blackboard_local.h>

namespace UPO
{
    class BehaviorTreeNode final
    {
        private:
            using PluginsService = behavior_tree_ros::GetLoadedPlugins;

        public:
            BehaviorTreeNode();
            ~BehaviorTreeNode() = default;

        private:
            bool GetLoadedPluginsService(PluginsService::Request& _request, PluginsService::Response& _response);

            void LoadPlugins(const std::string& _plugins_folder);
            void BuildTree(const std::string& _tree_file);

        private:
            ros::NodeHandle node_handle_;

            ros::ServiceServer get_loaded_plugins_srv_;

            BT::BehaviorTreeFactory bt_factory_;

            /*
            BT::StdCoutLogger   bt_logger_cout_;
            BT::FileLogger      bt_logger_file_;
            BT::MinitraceLogger bt_logger_trace_;
            BT::PublisherZMQ    bt_logger_zmq_;
            */

            std::set<std::string> loaded_plugins_;
    };
}

#endif
