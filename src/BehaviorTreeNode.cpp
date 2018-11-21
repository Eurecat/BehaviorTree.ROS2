#include <boost/filesystem.hpp>

#include "BehaviorTreeNode.hpp"

namespace UPO
{
    BehaviorTreeNode::BehaviorTreeNode() :
        loop_rate_ { node_handle_.param("tick_frequency", 30.0) }
    {
        node_handle_.getParam("trees_folder", trees_folder_);

        std::string plugins_folder;

        if(!node_handle_.getParam("plugins_folder", plugins_folder))
        {
            ROS_FATAL("Plugins folder param is missing. Aborting...");
            ros::shutdown();
        }

        LoadPlugins(plugins_folder);

        get_loaded_plugins_srv_ = node_handle_.advertiseService("behavior_tree/get_loaded_plugins", &BehaviorTreeNode::GetLoadedPluginsService, this);
        load_tree_srv_          = node_handle_.advertiseService("behavior_tree/load_tree", &BehaviorTreeNode::LoadTree, this);
        stop_tree_srv_          = node_handle_.advertiseService("behavior_tree/stop_tree", &BehaviorTreeNode::StopTree, this);
    }

    void BehaviorTreeNode::Loop()
    {
        if(!tree_)
        {
            loop_rate_.sleep();
            return;
        }

        const auto tree_status = tree_->Tick();
        if(tree_status == ROSTree::Status::FAILURE)
        {
            ROS_ERROR("Tree finished with errors");
            RemoveTree();
        }
        else if(tree_status == ROSTree::Status::SUCCESS)
        {
            ROS_INFO("Tree finished with no errors");
            RemoveTree();
        }

        loop_rate_.sleep();
    }

    //Private
    bool BehaviorTreeNode::LoadTree(LoadTreeService::Request& _request, LoadTreeService::Response& _response)
    {
        const auto& full_path = GetFullPath(_request.tree_file);

        try
        {
            BuildTree(full_path);
            ROS_INFO("Loaded tree %s", full_path.c_str());
        }
        catch(const std::runtime_error& ex)
        {
            ROS_ERROR("Error loading tree %s: %s", full_path.c_str(), ex.what());
            return false;
        }
        return true;
    }

    bool BehaviorTreeNode::StopTree(std_srvs::Empty::Request& _request, std_srvs::Empty::Response& _response)
    {
        RemoveTree();
        return true;
    }

    bool BehaviorTreeNode::GetLoadedPluginsService(PluginsService::Request& _request, PluginsService::Response& _response)
    {
        _response.plugins.assign(loaded_plugins_.cbegin(), loaded_plugins_.cend());
        return true;
    }

    void BehaviorTreeNode::BuildTree(const std::string& _tree_file)
    {
        tree_ = std::make_unique<ROSTree>(bt_factory_, _tree_file, node_handle_);
    }
    
    void BehaviorTreeNode::RemoveTree()
    {
        tree_.reset();
    }

    void BehaviorTreeNode::LoadPlugins(const std::string& _folder)
    {
        using namespace boost::filesystem;
        auto directory_list = [&] { return boost::make_iterator_range(directory_iterator(_folder), {}); };

        for(const auto& entry : directory_list())
        {
            if((!is_regular_file(entry) && !is_symlink(entry)) || entry.path().extension() != ".so") { continue; }

            try
            {
                const auto& plugin_path = canonical(entry.path());
                bt_factory_.registerFromPlugin(plugin_path.string());
                loaded_plugins_.emplace(plugin_path.filename().string());
                ROS_INFO("Loaded plugin %s", plugin_path.filename().string().c_str());
            }
            catch(const std::runtime_error& ex)
            {
                ROS_ERROR("Cannot load plugin %s. Error: %s", entry.path().filename().string().c_str(), ex.what());
            }
        }
    }

    std::string BehaviorTreeNode::GetFullPath(const std::string& _file) const
    {
        return _file.front() == '/' ? _file : (trees_folder_.back() == '/' ? trees_folder_ : trees_folder_ + "/") + _file;
    }
}
