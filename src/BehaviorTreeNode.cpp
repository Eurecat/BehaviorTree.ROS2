#include <chrono>
#include <sstream>
#include <iomanip>
#include <ctime>

#include <boost/filesystem.hpp>
#include <behaviortree_cpp/blackboard/blackboard_local.h>

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
        tree_ = std::make_unique<BT::Tree>(BT::buildTreeFromFile(bt_factory_, _tree_file, BT::Blackboard::create<BT::BlackboardLocal>()));
        InitializeLoggers();
    }
    
    void BehaviorTreeNode::RemoveTree()
    {
        ResetLoggers();
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

    void BehaviorTreeNode::InitializeLoggers()
    {
        if(!tree_) { return; }

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
            bt_logger_cout_ = std::make_unique<BT::StdCoutLogger>(tree_->root_node);
        }
        
        if(node_handle_.param("enable_minitrace_log", false))
        { 
            bt_logger_trace_ = std::make_unique<BT::MinitraceLogger>(tree_->root_node, minitrace_file.c_str());
        }
        
        if(node_handle_.param("enable_file_log", false))
        { 
            bt_logger_file_ = std::make_unique<BT::FileLogger>(tree_->root_node, log_file.c_str());
        }

        if(node_handle_.param("enable_zmq_log", false))
        {
            #ifdef ZMQ_FOUND
            bt_logger_zmq_ = std::make_unique<BT::PublisherZMQ>(tree_->root_node);
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
        #ifdef ZMQ_FOUND
        bt_logger_zmq_.reset();
        #endif
    }
}
