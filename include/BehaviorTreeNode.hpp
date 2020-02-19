#ifndef BEHAVIOR_TREE_NODE_HPP
#define BEHAVIOR_TREE_NODE_HPP

#include <set>
#include <string>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <behavior_tree_ros/GetLoadedPlugins.h>
#include <behavior_tree_ros/LoadTree.h>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>

#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_file_logger.h>
#include <behaviortree_cpp/loggers/bt_minitrace_logger.h>

#ifdef BEHAVIOR_TREE_CPP_ZMQ
#include <behaviortree_cpp/loggers/bt_zmq_publisher.h>
#endif

#include "bt_rostopic_logger.h"


namespace UPO
{
    class BehaviorTreeNode final
    {
        private:
            using PluginsService  = behavior_tree_ros::GetLoadedPlugins;
            using LoadTreeService = behavior_tree_ros::LoadTree;

        public:
            BehaviorTreeNode();
            ~BehaviorTreeNode() = default;

            void Loop();

        private:
            bool GetLoadedPluginsService(PluginsService::Request& _request, PluginsService::Response& _response);
            bool LoadTree(LoadTreeService::Request& _request, LoadTreeService::Response& _response);
            bool StopTree(std_srvs::Empty::Request& _request, std_srvs::Empty::Response& _response);

            void LoadAllPlugins();
            void LoadPluginsFromROS();
            void LoadPluginsFromFolder(const std::string& _plugins_folder);
            void LoadPlugin(const std::string& _plugin_path);

            void BuildTree(const std::string& _xml_file);
            void RemoveTree();

            void InitializeLoggers();
            void ResetLoggers();

            std::string GetFullPath(const std::string& _file) const;

        private:
            ros::NodeHandle node_handle_;
            ros::Rate loop_rate_;

            ros::ServiceServer get_loaded_plugins_srv_;
            ros::ServiceServer load_tree_srv_;
            ros::ServiceServer stop_tree_srv_;

	    ros::Publisher bt_status_publisher_;

            std::unique_ptr<BT::Tree> tree_;
            BT::BehaviorTreeFactory bt_factory_;

            std::unique_ptr<BT::StdCoutLogger>   bt_logger_cout_;
            std::unique_ptr<BT::FileLogger>      bt_logger_file_;
            std::unique_ptr<BT::MinitraceLogger> bt_logger_trace_;
            #ifdef BEHAVIOR_TREE_CPP_ZMQ
            std::unique_ptr<BT::PublisherZMQ>    bt_logger_zmq_;
            #endif

	    std::unique_ptr<BT_ROS::RosTopicLogger> bt_logger_rostopic_;

            std::set<std::string> loaded_plugins_;
            std::string trees_folder_;
    };
}

#endif
