#include <chrono>
#include <sstream>
#include <iomanip>
#include <ctime>

#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <tinyxml2.h>

#include "BehaviorTreeNode.hpp"

namespace UPO
{
    BehaviorTreeNode::BehaviorTreeNode() :
        loop_rate_ { node_handle_.param("tick_frequency", 30.0) }
    {
        node_handle_.getParam("trees_folder", trees_folder_);

        LoadAllPlugins();

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

        try
        {
            const auto tree_status = tree_->root_node->executeTick();

            if(tree_status == BT::NodeStatus::FAILURE)
            {
                ROS_ERROR("Tree finished with errors");
                RemoveTree();
            }
            else if(tree_status == BT::NodeStatus::SUCCESS)
            {
                ROS_INFO("Tree finished with no errors");
                RemoveTree();
            }
        }
        catch(const BT::BehaviorTreeException& ex)
        {
            ROS_ERROR("Tree crashed with exception: %s", ex.what());
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
        // Wait between creating and executing the Tree to fully initialize ROS publishers
        auto temp_tree = std::make_unique<BT::Tree>(bt_factory_.createTreeFromFile(_tree_file));
        ros::Duration(0.5).sleep();
        tree_.swap(temp_tree);
        InitializeLoggers();
    }
    
    void BehaviorTreeNode::RemoveTree()
    {
        ResetLoggers();
        tree_.reset();
    }

    void BehaviorTreeNode::LoadPluginsFromROS()
    {
        using namespace tinyxml2;

        // ros::package::getPlugins returns a pair of strings for each result.
        // The first one is the name of the package that exported the target xml entry (behavior_tree_ros)
        // and the second one is the value of the attribute (plugin)
        std::vector<std::pair<std::string, std::string>> exported_plugins;
        ros::package::getPlugins("behavior_tree_ros", "plugin", exported_plugins);
        XMLDocument plugin_description;

        for(const auto& plugin : exported_plugins)
        {
            try
            {
                plugin_description.LoadFile(plugin.second.c_str());

                if(plugin_description.Error())
                {
                    std::string error_msg;
                    #ifdef MELODIC
                    error_msg  =std::string { "XML file may be ill-formed ( " }
                            + plugin_description.ErrorStr() ;
                    #endif

                    #ifndef MELODIC
                    error_msg = std::string { "XML file may be ill-formed ( " }
                            + plugin_description.GetErrorStr1() + ". "
                            + plugin_description.GetErrorStr2() + ")";
                    #endif
                    throw std::runtime_error { error_msg };
                }

                XMLElement* root_entry = plugin_description.RootElement(); 

                if(!root_entry)
                {
                    throw std::runtime_error { "No root element was found in XML file" };
                }

                XMLElement* plugin_entry = root_entry->FirstChildElement("plugin");

                // This loop abort the parsing on first error. This could be a problem if there are multiple plugins
                // defined in the same file.
                while(plugin_entry)
                {
                    std::string plugin_lib = plugin_entry->Attribute("path");

                    if(plugin_lib.empty())
                    {
                        throw std::runtime_error { "Missing path attribute in plugin element" };
                    }

                    std::string devel_path;
                    const std::string& xml_path = plugin.second;

                    // Is there a better way to do this?
                    // Check if the path contains a src folder (then we assume is a local workspace)
                    // or if it contains a share folder (then we assume we are in the system path)
                    if(xml_path.find("/src/") != std::string::npos)
                    {
                        devel_path = xml_path.substr(0, xml_path.find("/src/")) + "/devel/";
                    }
                    else if(xml_path.find("/share/") != std::string::npos)
                    {
                        devel_path = xml_path.substr(0, xml_path.find("/share/")) + "/";
                    }

                    if(devel_path.empty())
                    {
                        throw std::runtime_error { "Cannot find devel path to plugin" };
                    }

                    std::string lib_full_path = devel_path + plugin_lib + ".so";
                    LoadPlugin(lib_full_path);

                    ROS_INFO("Loaded plugin %s from ROS plugin", lib_full_path.c_str());

                    plugin_entry = plugin_entry->NextSiblingElement("plugin");
                }
            }
            catch(const std::runtime_error& ex)
            {
                ROS_ERROR("Error loading plugin %s in path %s: %s.", plugin.first.c_str(),
                        plugin.second.c_str(), ex.what());
            }
        }
    }

    void BehaviorTreeNode::LoadPluginsFromFolder(const std::string& _folder)
    {
        using namespace boost::filesystem;

        if(!exists(_folder))
        {
            ROS_ERROR("Plugin folder %s does not exist.", _folder.c_str());
	        return;
        }

        auto directory_list = [&] { return boost::make_iterator_range(directory_iterator(_folder), {}); };

        for(const auto& entry : directory_list())
        {
            if((!is_regular_file(entry) && !is_symlink(entry)) || entry.path().extension() != ".so") { continue; }

            try
            {
                const auto& plugin_path = canonical(entry.path());
		        LoadPlugin(plugin_path.string());
                ROS_INFO("Loaded plugin %s from folder %s", plugin_path.filename().string().c_str(),
				_folder.c_str());
            }
            catch(const std::runtime_error& ex)
            {
                ROS_ERROR("Cannot load plugin %s from folder %s. Error: %s", entry.path().filename().string().c_str(),
				_folder.c_str(), ex.what());
            }
        }
    }

    void BehaviorTreeNode::LoadPlugin(const std::string& _plugin_path)
    {
        try
        {
	        bt_factory_.registerFromPlugin(_plugin_path);
            loaded_plugins_.emplace(_plugin_path);
	    }
        catch(const BT::BehaviorTreeException& ex)
        {
	        throw std::runtime_error { ex.what() };
        }
    }

    void BehaviorTreeNode::LoadAllPlugins()
    {
        LoadPluginsFromROS();

	bool import_from_folder = node_handle_.param("import_from_folder", false);

	if(import_from_folder)
    {
        std::string plugins_folder;
        if(!node_handle_.getParam("plugins_folder", plugins_folder))
        {
            ROS_WARN("Import from folder option is set, but folder param is missing");
        }
        else
        {
            LoadPluginsFromFolder(plugins_folder);
        }
    }
    }

    std::string BehaviorTreeNode::GetFullPath(const std::string& _file) const
    {
        return _file.front() == '/' ? _file : (trees_folder_.back() == '/' ? trees_folder_ : trees_folder_ + "/") + _file;
    }

    void BehaviorTreeNode::InitializeLoggers()
    {
        if(!tree_ || !tree_->root_node) { return; }

        //Behaviortree_cpp complains if two instances of the same logger exist at the same time,
        //so the pointer is resetted explictly first
        ResetLoggers();

        std::stringstream file_base;
        const auto& current_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

        file_base << node_handle_.param<std::string>("log_folder", "/tmp/") << "behavior_tree_ros-" << std::put_time(std::localtime(&current_time), "%F-%R");;
        const auto& log_file       = file_base.str() + ".fbl";
        const auto& minitrace_file = file_base.str() + ".json";

        if(node_handle_.param("enable_cout_log", false))
        { 
            bt_logger_cout_ = std::make_unique<BT::StdCoutLogger>(*tree_);
        }
        
        if(node_handle_.param("enable_minitrace_log", false))
        { 
            bt_logger_trace_ = std::make_unique<BT::MinitraceLogger>(*tree_, minitrace_file.c_str());
        }
        
        if(node_handle_.param("enable_file_log", false))
        { 
            bt_logger_file_ = std::make_unique<BT::FileLogger>(*tree_, log_file.c_str());
        }

        if(node_handle_.param("enable_zmq_log", false))
        {
            #ifdef BEHAVIOR_TREE_CPP_ZMQ
            bt_logger_zmq_ = std::make_unique<BT::PublisherZMQ>(*tree_);
            #else
            ROS_WARN("ZMQ logging is enabled but behaviortree_cpp was not compiled with ZMQ support.");
            #endif
        }

    }

    void BehaviorTreeNode::ResetLoggers()
    {
        bt_logger_cout_.reset();
        bt_logger_trace_.reset();
        bt_logger_file_.reset();
        #ifdef BEHAVIOR_TREE_CPP_ZMQ
        bt_logger_zmq_.reset();
        #endif
    }
}
