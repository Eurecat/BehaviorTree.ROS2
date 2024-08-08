#include <ros/package.h>
#include <boost/filesystem.hpp>

#include "BehaviorTreeNode.hpp"
#include "behavior_tree_ros/TreeExecutionStatus.h"
#include "behavior_tree_ros/3rdparty/tinyxml2/tinyxml2.h"
#include <thread>

namespace BT_ROS
{
    BehaviorTreeNode::BehaviorTreeNode() :
        loop_rate_(private_node_handle_.param("tick_frequency", 30.0)),
        bt_action_server_(public_node_handle_, "behavior_tree/load_tree_action", false)
    {
        ROS_INFO("INSTANTIATING BTNODE");
        private_node_handle_.getParam("trees_folder", trees_folder_);

        enable_cout_log_        = private_node_handle_.param("enable_cout_log", false);
        enable_minitrace_log_   = private_node_handle_.param("enable_minitrace_log", false);
        enable_rostopic_log_    = private_node_handle_.param("enable_rostopic_log", false);
        enable_file_log_        = private_node_handle_.param("enable_file_log", false);
        enable_zmq_log_         = private_node_handle_.param("enable_zmq_log", false);

        //LOAD TREE Parameters
        int uid = 0;
        int server_port = 0;
        int publisher_port = 0;

        std::string bb_init = "";
        private_node_handle_.param<std::string>("tree_name",service_tree_.tree_name_, "");
        private_node_handle_.param<std::string>("tree_file",service_tree_.tree_filename_, "");
        private_node_handle_.param<std::string>("tree_bb_init",bb_init, "");
        private_node_handle_.param<int>("tree_uid",uid, 1);
        service_tree_.tree_debug_ = private_node_handle_.param("tree_debug", false);
        private_node_handle_.param<int>("server_port", server_port, 1667);
        private_node_handle_.param<int>("publisher_port", publisher_port,1666);

        bb_init.erase(std::remove(bb_init.begin(), bb_init.end(),'['), bb_init.end());
        bb_init.erase(std::remove(bb_init.begin(), bb_init.end(),']'), bb_init.end());
        
        std::stringstream bb_init_stream(bb_init);
        std::string s;
        while (getline(bb_init_stream, s, ',')) {
            // store token string in the vector
            service_tree_.tree_bb_init_.push_back(s);
        }   

        const char* home = getenv("HOME");
        log_folder_ = private_node_handle_.param<std::string>("log_folder", "/tmp/");
        log_folder_ = log_folder_.front() == '~' ? std::string(home) + log_folder_.substr(1, log_folder_.size() - 1) : log_folder_;

        LoadAllPlugins();

        get_loaded_plugins_srv_ = public_node_handle_.advertiseService("/"+service_tree_.tree_name_+"/get_loaded_plugins", &BehaviorTreeNode::GetLoadedPluginsService, this);
        stop_tree_srv_          = public_node_handle_.advertiseService("/"+service_tree_.tree_name_+"/stop_tree", &BehaviorTreeNode::StopTree, this);
        get_tree_status_srv_    = public_node_handle_.advertiseService("/"+service_tree_.tree_name_+"/status_tree", &BehaviorTreeNode::StatusTree, this);
        
        if(enable_rostopic_log_)
        {
           service_tree_.InitializeStatusPublisher(public_node_handle_,service_tree_.tree_name_);
           action_tree_.InitializeStatusPublisher(public_node_handle_,"action_tree");
        }

        // Set action callbacks
        bt_action_server_.registerGoalCallback(boost::bind(&BehaviorTreeNode::ActionGoalCB, this));
        bt_action_server_.registerPreemptCallback(boost::bind(&BehaviorTreeNode::ActionPreemptCB, this));

        // Start action
        bt_action_server_.start();

        if ( (uid >= 0) && (server_port >= 0) && (publisher_port >= 0) )
        {
            service_tree_.publisher_port_ = publisher_port;
            service_tree_.server_port_ = server_port ;
            service_tree_.tree_uid_ = uid;

             std::cout << "LOADING TREE " << std::endl;

            if (!LoadTree())
            {
                ROS_ERROR("ERROR: FAILED TO LOAD THE TREE");
            }
        }
        else
        {
            ROS_ERROR("ERROR: UID, SERVER_PORT and PUBLISHER_PORT CAN'T HAVE NEGATIVE VALUES");
        }
    }

    void BehaviorTreeNode::Loop()
    {
        // Sleep if no tree running (main and remote)
        if(!service_tree_.IsTreeLoaded() && !action_tree_.IsTreeLoaded())
        {
            loop_rate_.sleep();
            return;
        }

        if(service_tree_.IsTreeLoaded() && service_tree_.AreLoggersInitialized()) { // Tick main tree (loaded with service)
            try
            {
                const auto tree_status = service_tree_.tickTree();
                
                // Publish the updated status
                if(tree_status == BT::NodeStatus::FAILURE)
                {
                    service_tree_.execution_tree_status_ = "FINISHED";
                    service_tree_.execution_tree_error_ = "FAILURE";
                    service_tree_.status_ = tree_status;
                    service_tree_.PublishExecutionStatus();
                    ROS_ERROR("Tree finished with errors");
                    RemoveTree();
                }
                else if(tree_status == BT::NodeStatus::SUCCESS)
                {
                    service_tree_.execution_tree_status_ = "FINISHED";
                    service_tree_.execution_tree_error_ = "SUCCESS";
                    service_tree_.status_ = tree_status;
                    service_tree_.PublishExecutionStatus();
                    ROS_INFO("Tree finished with no errors");
                    RemoveTree();
                }
                //IDLE --> RUNNING --> PAUSED
                else if (tree_status != service_tree_.status_)
                {
                    service_tree_.status_ = tree_status;
                    service_tree_.PublishExecutionStatus();
                }
            }
            catch(const BT::BehaviorTreeException& ex)
            {
                std::string error_str = "ERROR: Tree crashed with exception [ " + std::string(ex.what()) + " ]";
                ROS_ERROR("Tree crashed with exception: %s", ex.what());
                service_tree_.execution_tree_status_ = "CRASHED";
                service_tree_.execution_tree_error_ = ex.what() ;
                service_tree_.PublishExecutionStatus(true, error_str);
                RemoveTree();
            }
        }

        if(action_tree_.IsTreeLoaded() && action_tree_.AreLoggersInitialized()) { // Tick remote tree (loaded with action)
            try
            {
                const auto action_tree_status = action_tree_.tickTree();

                action_feedback_.status.data = "RUNNING";
                bt_action_server_.publishFeedback(action_feedback_);

                if(action_tree_status == BT::NodeStatus::FAILURE)
                {
                    ROS_ERROR("Action tree finished with errors");
                    action_tree_.RemoveTree();
                    action_result_.result = false;
                    bt_action_server_.setAborted(action_result_);
                }
                else if(action_tree_status == BT::NodeStatus::SUCCESS)
                {
                    ROS_INFO("Action tree finished with no errors");
                    action_tree_.RemoveTree();
                    action_result_.result = true;
                    bt_action_server_.setSucceeded(action_result_);
                }
            }
            catch(const BT::BehaviorTreeException& ex)
            {
                ROS_ERROR("Action tree crashed with exception: %s", ex.what());
                action_tree_.RemoveTree();
            }
        }

        loop_rate_.sleep();
    }

    //Private
    bool BehaviorTreeNode::LoadTree()
    {
        const auto& full_path = GetFullPath(service_tree_.tree_filename_);

        service_tree_.execution_time_ = ros::Time::now();
        service_tree_.status_ = BT::NodeStatus::IDLE;
        service_tree_.PublishExecutionStatus();

        //Init error in case of CRASH
        service_tree_.execution_tree_status_ = "ERROR LOADING";
        service_tree_.execution_tree_error_ = "COULD NOT LOAD";

        try
        {
            // If there's a tree being executed, halt and destoy it to execute the new one
            if (service_tree_.IsTreeLoaded()) { service_tree_.RemoveTree(); }

            // Note: I'm saving the tree_file instead of
            // the full path to be consistent with the original request.
            std::cout << "BUILDING TREE ... " << std::endl;
            service_tree_.BuildTree(full_path, bt_factory_, service_tree_.tree_debug_, service_tree_.tree_bb_init_);
             std::cout << "BUILD TREE OK " << std::endl;
            service_tree_.InitializeLoggers(enable_cout_log_, enable_minitrace_log_, enable_file_log_, enable_rostopic_log_, enable_zmq_log_, log_folder_);
            std::cout << "INIT LOGGERS OK " << std::endl;
            ROS_INFO("Loaded srv tree %s counting of %ld nodes", full_path.c_str(), service_tree_.TreeNodesCount());
        }
        catch(const std::runtime_error& ex)
        {
            ROS_ERROR("Error loading tree %s: %s", full_path.c_str(), ex.what());

            std::string error_str = "Error loading tree " + full_path + " " + std::string(ex.what());;
            //current_tree_.clear();
            service_tree_.execution_tree_status_ = "ERROR LOADING";
            service_tree_.execution_tree_error_ = ex.what() ;

            service_tree_.PublishExecutionStatus(true,error_str);
            return false;
        }

        service_tree_.execution_tree_status_ = "IDLE";
        service_tree_.execution_tree_error_ = "";
        return true;
    }


    bool BehaviorTreeNode::StopTree(std_srvs::Empty::Request& _request, std_srvs::Empty::Response& _response)
    {
        service_tree_.execution_tree_status_ = "FINISHED";
        service_tree_.execution_tree_error_ = "Canceled by StopTree Service";
        service_tree_.status_ = BT::NodeStatus::FAILURE;
        service_tree_.PublishExecutionStatus();
        RemoveTree();
        return true;
    }

    bool BehaviorTreeNode::StatusTree(StatusService::Request& _request, StatusService::Response& _response)
    {
        behavior_tree_ros::TreeExecutionStatus tree_status_msg;

        //Fill Service Tree info
        tree_status_msg.status = service_tree_.execution_tree_status_;
        tree_status_msg.name = service_tree_.tree_name_;
        tree_status_msg.file = service_tree_.tree_filename_;
        tree_status_msg.time_start = service_tree_.execution_time_;
        tree_status_msg.time = ros::Time::now();
        tree_status_msg.uid = service_tree_.tree_uid_;
        tree_status_msg.data = service_tree_.execution_tree_error_;
        _response.status = tree_status_msg;
        return true;
    }
    bool BehaviorTreeNode::GetLoadedPluginsService(PluginsService::Request& _request, PluginsService::Response& _response)
    {
        _response.plugins.assign(loaded_plugins_.cbegin(), loaded_plugins_.cend());
        return true;
    }

    void BehaviorTreeNode::RemoveTree()
    {
        service_tree_.RemoveTree();

        // Update and publish the status here too
        // (neeed to cover the case where users manually
        // stop a tree by calling the stop_tree service).
        //PublishExecutionStatus();
    }

    void BehaviorTreeNode::LoadPluginsFromROS()
    {
        using namespace BT_TinyXML2;

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
                    error_msg  =std::string { "XML file may be ill-formed ( " }
                            + plugin_description.ErrorStr() ;
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

        bool import_from_folder = private_node_handle_.param("import_from_folder", false);

        if(import_from_folder)
        {
            std::string plugins_folder;
            if(!private_node_handle_.getParam("plugins_folder", plugins_folder))
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
        std::string full_name = _file;
        const char* home = getenv("HOME");

        full_name = full_name.front() == '~' ? std::string(home) + full_name.substr(1, full_name.size() - 1) : full_name;
        full_name = full_name.front() == '/' ? full_name : (trees_folder_.back() == '/' ? trees_folder_ : trees_folder_ + "/") + full_name;

        std::size_t pos = full_name.find_last_of('.');
        if (pos == std::string::npos)
            full_name += ".xml";
        else if (full_name.substr(pos) != ".xml")
            full_name += ".xml";

        return full_name;
    }

    // Publishes the current execution status. Note that
    // this method assumes the member variables "status_"
    // and "current_tree_" are up to date.

    void BehaviorTreeNode::ActionGoalCB()
    {
        const auto goal = bt_action_server_.acceptNewGoal();
        const std::string full_path = GetFullPath(goal->tree_file.data);

        try
        {
            ROS_INFO("Loading action tree %s", full_path.c_str());
            std::vector<std::string> initbb_yaml_filepaths{};
            for(auto initbbyaml_path_it = goal->bb_init_files.begin(); initbbyaml_path_it != goal->bb_init_files.end(); initbbyaml_path_it++)
            {
                initbb_yaml_filepaths.push_back(std::string{initbbyaml_path_it->data});
            }
            action_tree_.BuildTree(full_path, bt_factory_, goal->debug, initbb_yaml_filepaths);

            // If the service tree is loaded it means that the action was called from the remote BT block
            // As such, don't show status messages through the terminal
            if(service_tree_.IsTreeLoaded())
                action_tree_.InitializeLoggers(false, enable_minitrace_log_, enable_file_log_, enable_rostopic_log_, enable_zmq_log_, log_folder_);
            else
                action_tree_.InitializeLoggers(enable_cout_log_, enable_minitrace_log_, enable_file_log_, enable_rostopic_log_, enable_zmq_log_, log_folder_);

            ROS_INFO("Loaded action tree %s counting of %ld nodes", full_path.c_str(), service_tree_.TreeNodesCount());
        }
        catch(const std::runtime_error& ex)
        {
            ROS_ERROR("Error loading tree %s: %s", full_path.c_str(), ex.what());
            bt_action_server_.setAborted();
        }

        // Preempts received for the new goal between checking if isNewGoalAvailabel
        // or invocation of a goal callback and the acceptNewGoal call will not trigger a preempt callback.
        // This means, isPreemptRequested should be called after accepting the goal even for
        // callback-based implementations to make sure the new goal does not have a pending preempt request.
        if(bt_action_server_.isPreemptRequested())
            bt_action_server_.setPreempted();
    }

    void BehaviorTreeNode::ActionPreemptCB()
    {
        action_tree_.RemoveTree();
        action_result_.result = false;
        bt_action_server_.setPreempted(action_result_);
    }
}
