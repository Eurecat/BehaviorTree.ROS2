#include <boost/filesystem.hpp>
#include <behavior_tree_core/xml_parsing.h>

#include <Blackboard/blackboard_local.h>

#include "BehaviorTreeNode.hpp"

namespace UPO
{
    BehaviorTreeNode::BehaviorTreeNode()
    {
        std::string plugins_folder;

        if(!node_handle_.getParam("plugins_folder", plugins_folder))
        {
            ROS_FATAL("Plugins folder param is missing. Aborting...");
            ros::shutdown();
        }

        LoadPlugins(plugins_folder);

        get_loaded_plugins_srv_ = node_handle_.advertiseService("behavior_tree/get_loaded_plugins", &BehaviorTreeNode::GetLoadedPluginsService, this);
    }

    //Private
    void BehaviorTreeNode::BuildTree(const std::string& _tree_file)
    {
        //tree = BT::buildTreeFromFile(factory_, _tree_file, BT::Blackboard::create<BT::BlackboardLocal>());
    }

    bool BehaviorTreeNode::GetLoadedPluginsService(PluginsService::Request& _request, PluginsService::Response& _response)
    {
        _response.plugins.assign(loaded_plugins_.cbegin(), loaded_plugins_.cend());
        return true;
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
}
