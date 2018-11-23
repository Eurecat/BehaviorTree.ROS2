#ifndef BEHAVIOR_TREE_NODE_HPP
#define BEHAVIOR_TREE_NODE_HPP

#include <set>
#include <string>
#include <memory>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <behavior_tree_ros/GetLoadedPlugins.h>
#include <behavior_tree_ros/LoadTree.h>

#include <behaviortree_cpp/bt_factory.h>

#include "ROSTree.hpp"

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

            void LoadPlugins(const std::string& _plugins_folder);

            void BuildTree(const std::string& _xml_file);
            void RemoveTree();

            std::string GetFullPath(const std::string& _file) const;

        private:
            ros::NodeHandle node_handle_;
            ros::Rate loop_rate_;

            ros::ServiceServer get_loaded_plugins_srv_;
            ros::ServiceServer load_tree_srv_;
            ros::ServiceServer stop_tree_srv_;

            std::unique_ptr<ROSTree> tree_;
            BT::BehaviorTreeFactory bt_factory_;

            std::set<std::string> loaded_plugins_;
            std::string trees_folder_;
    };
}

#endif
