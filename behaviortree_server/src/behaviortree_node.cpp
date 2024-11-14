
#include "behaviortree_node.hpp"
#include "yaml-cpp/yaml.h"


    BehaviorTreeNode::BehaviorTreeNode(const rclcpp::Node::SharedPtr& node) : tree_wrapper_(node), node_(node) 
    {
      //Fetch Tree Parameters
      getParameters(node);

      //Load Plugins
      LoadAllPlugins();
      
      //Create Tree Services
      RCLCPP_INFO(node_->get_logger(),"CREATING SERVICES");
      get_loaded_plugins_srv_ =node->create_service<GetLoadedPluginsSrv>("/"+tree_name_+"/get_loaded_plugins",std::bind(&BehaviorTreeNode::GetLoadedPluginsService,this,_1,_2));
      stop_tree_srv_ = node_->create_service<EmptySrv>("/"+tree_name_+"/stop_tree",std::bind(&BehaviorTreeNode::StopTree,this,_1,_2));
      restart_tree_srv_ = node_->create_service<EmptySrv>("/"+tree_name_+"/restart_tree",std::bind(&BehaviorTreeNode::RestartTree,this,_1,_2));
      get_tree_status_srv_ = node_->create_service<GetTreeStatusSrv>("/"+tree_name_+"/status_tree",std::bind(&BehaviorTreeNode::StatusTree,this,_1,_2));
      RCLCPP_INFO(node_->get_logger(),"CREATING SERVICES OK");

      InitializeLoggers();

      //Create Blackboard and tree
      if ( (tree_uid_ >= 0) && (tree_server_port_ >= 0) && (tree_publisher_port_ >= 0) )
      {
        RCLCPP_INFO(node_->get_logger(),"CREATING BB");
        tree_wrapper_.params_.groot2_port = tree_server_port_;

        if (tree_bb_init_.size() > 0)
        {
          RCLCPP_INFO(node_->get_logger(),"INIT BB");
          for(const auto& bb_init_abs_filepath: tree_bb_init_)
          {
              if(bb_init_abs_filepath.length() < 3) continue;
              InitializeBlackboard(bb_init_abs_filepath, tree_wrapper_.globalBlackboard(), false);
          }
          RCLCPP_INFO(node_->get_logger(),"INIT BB OK");
        }

        RCLCPP_INFO(node_->get_logger(),"CREATING BB OK");

        const auto& full_path = GetFullPath(tree_filename_);

        RCLCPP_INFO(node_->get_logger(),"CREATING TREE FROM FILE %s", full_path.c_str());
        return;
        tree_wrapper_.tree_ = tree_wrapper_.factory_.createTreeFromFile(full_path,tree_wrapper_.global_blackboard_);
        RCLCPP_INFO(node_->get_logger(),"CREATED TREE FROM FILE OK");

        //if (tree_debug_) tree_wrapper_.tree_.setDebug();

        /*sendBlackboardUpdates(tree_wrapper_.tree_.getKeysValueToSync()); // send updates
        getBlackboardUpdates(); // blocking call to update bb with missing values that need to be retrieved from server
        */


        start_execution_time_ = node_->get_clock()->now();
        tree_wrapper_.is_tree_loaded_ = true;

        RCLCPP_INFO(node_->get_logger(),"TREE LOADED OK");
      }
      else
      {
          RCLCPP_ERROR(node_->get_logger(),"ERROR: UID, SERVER_PORT and PUBLISHER_PORT CAN'T HAVE NEGATIVE VALUES");
          return;
      }

      RCLCPP_INFO(node_->get_logger(),"CREATE BT_SERVER PUB & SUB");
      //Updates subscriber server side
      sync_bb_sub_ = node_->create_subscription<BBEntry>("behavior_tree_server/broadcast_update", 10, std::bind(&BehaviorTreeNode::SyncBlackboardUpdateCallback, this, _1)) ;
      //Updates republisher for all trees (put latch to true atm, because seems a good option that you receive last update from the server)
      sync_bb_pub_ = node_->create_publisher<BBEntry>("/behavior_tree_server/local_update", 10);

      RCLCPP_INFO(node_->get_logger(),"CREATE BT_SERVER PUB & SUB OK");
    }

    void BehaviorTreeNode::InitializeLoggers()
    {
      //Create Loggers
      if(enable_rostopic_log_)
      {
        RCLCPP_INFO(node_->get_logger(),"INIT ROSTOPIC LOGS");
        InitializeStatusPublisher(tree_name_);
        RCLCPP_INFO(node_->get_logger(),"INIT ROSTOPIC LOGS OK");
      }

      if (enable_zmq_log_) 
      {
        #ifdef BEHAVIOR_TREE_CPP_ZMQ
        groot_publisher_.reset();
        groot_publisher_ = std::make_shared<BT::Groot2Publisher>(tree_wrapper_.tree_, tree_server_port_);
        #else
          RCLCPP_WARN(node_->get_logger(),"ZMQ logging is enabled but behavior_tree_core was not compiled with ZMQ support.");
        #endif
      }

      loggers_init_ = true;
    }

    bool BehaviorTreeNode::GetLoadedPluginsService(const std::shared_ptr<GetLoadedPluginsSrv::Request> _request, std::shared_ptr<GetLoadedPluginsSrv::Response> _response)
    {
      _response->plugins.assign(loaded_plugins_.cbegin(), loaded_plugins_.cend());
      return true;
    }
    bool BehaviorTreeNode::StopTree(const std::shared_ptr<EmptySrv::Request> _request, std::shared_ptr<EmptySrv::Response> _response)
    {
      tree_wrapper_.execution_tree_status_ = "FINISHED";
      tree_wrapper_.execution_tree_error_ = "Canceled by StopTree Service";
      tree_wrapper_.status_ = BT::NodeStatus::IDLE;
      PublishExecutionStatus();
      tree_wrapper_.ResetTree();
      return true;
    }
    bool BehaviorTreeNode::RestartTree(const std::shared_ptr<EmptySrv::Request> _request, std::shared_ptr<EmptySrv::Response> _response)
    {
      if (tree_wrapper_.is_tree_loaded_)
      {
        tree_wrapper_.SetExecuted(false);
        return true;
      }
      return false;
    }
    bool BehaviorTreeNode::StatusTree(const std::shared_ptr<GetTreeStatusSrv::Request> _request, std::shared_ptr<GetTreeStatusSrv::Response> _response)
    {
      TreeStatus tree_status_msg;

      //Fill Service Tree info
      tree_status_msg.status = tree_wrapper_.execution_tree_status_;
      tree_status_msg.name = tree_name_;
      tree_status_msg.file = tree_filename_;
      tree_status_msg.time_start = start_execution_time_;
      tree_status_msg.time = node_->get_clock()->now();;
      tree_status_msg.uid = tree_uid_;
      tree_status_msg.data = tree_wrapper_.execution_tree_error_;
      _response->status = tree_status_msg;
      return true;
    }
    
    void BehaviorTreeNode::SyncBlackboardUpdateCallback(const BBEntry::SharedPtr _topic_msg)
    {   
      //TODO
    }
    void BehaviorTreeNode::InitializeStatusPublisher(std::string tree_name)
    {
        bt_transition_publisher_ = node_->create_publisher<Transition>("/"+tree_name_+"/transition_status", 1);
        bt_execution_status_publisher_ = node_->create_publisher<TreeStatus>("/"+tree_name_+"/execution_status", 100);
    }

    /*void sendBlackboardUpdates(const BT::Blackboard::SerializedEntriesMap& entries_map)
    {
        // std::cout << "send BB UPDATES for tree " << service_tree_.tree_name_ << " " << std::to_string(entries_map.size()) << " \n" << std::flush;
        for(const auto ser_entry : entries_map)
        {
            BBEntry bb_entry_msg;
            bb_entry_msg.key = ser_entry.first;
            bb_entry_msg.type = ser_entry.second.first;
            bb_entry_msg.value = ser_entry.second.second;

            sync_bb_pub_->publish(bb_entry_msg);
        }
    }*/

    /*void getBlackboardUpdates(const bool just_empty_values)
    {
        rclcpp::Client<GetBBValues>::SharedPtr client = node_->create_client<GetBBValues>("/behavior_tree_server/get_sync_bb_values"); 
        GetBBValues::Request request;
        GetBBValues::Response response;
        const std::unordered_set<std::string> keys = getSyncKeys(just_empty_values);
        request.keys = std::vector<std::string>(keys.begin(), keys.end());
        if(client.call(request, response))
        {
            SyncBlackboardUpdateCallback(response.entries, &tree_wrapper_.factory());
        }
    }*/
    /* std::unordered_set<std::string> getSyncKeys(const bool just_empty_values)
    {
      return tree_wrapper_.globalBlackboard()->getSyncKeys(just_empty_values);
    }*/

    bool BehaviorTreeNode::AreLoggersInitialized()
    {
      return loggers_init_;
    }
    void BehaviorTreeNode::Loop()
    {
        // Sleep if no tree running (main and remote)
        if(!tree_wrapper_.IsTreeLoaded())
        {
          RCLCPP_INFO(node_->get_logger(),"TREE NOT LOADED -- END");
          rclcpp::shutdown();
          return;
        }

        if(AreLoggersInitialized() && !tree_wrapper_.HasExecutionTerminated()) { 
            try
            {
                RCLCPP_INFO(node_->get_logger(),"TICK ONCE");
                const auto tree_status = tree_wrapper_.tree_.tickOnce();
                RCLCPP_INFO(node_->get_logger(),"TICK ONCE OK");
                //TODO
                //sendBlackboardUpdates(tree_wrapper_.getKeysValueToSync());
                
                // Publish the updated status
                if(tree_status == BT::NodeStatus::FAILURE)
                {
                    tree_wrapper_.execution_tree_status_ = "FINISHED";
                    tree_wrapper_.execution_tree_error_ = "FAILURE";
                    tree_wrapper_.status_ = tree_status;
                    PublishExecutionStatus();
                    RCLCPP_ERROR(node_->get_logger(),"Tree finished with errors");
                    tree_wrapper_.ResetTree();// RemoveTree();
                    tree_wrapper_.SetExecuted(!tree_auto_restart_); // if auto restart is false, set executed to true to stop the tick, otherwise will restart the tick from the beginning
                }
                else if(tree_status == BT::NodeStatus::SUCCESS)
                {
                    tree_wrapper_.execution_tree_status_ = "FINISHED";
                    tree_wrapper_.execution_tree_error_ = "SUCCESS";
                    tree_wrapper_.status_ = tree_status;
                    PublishExecutionStatus();
                    RCLCPP_ERROR(node_->get_logger(),"Tree finished with no errors");
                    tree_wrapper_.ResetTree();// RemoveTree();
                    tree_wrapper_.SetExecuted(!tree_auto_restart_); // if auto restart is false, set executed to true to stop the tick, otherwise will restart the tick from the beginning
                }
                //IDLE --> RUNNING --> PAUSED
                else if (tree_status != tree_wrapper_.status_)
                {
                    tree_wrapper_.status_ = tree_status;
                    PublishExecutionStatus();
                }
            }
            catch(const BT::BehaviorTreeException& ex)
            {
                std::string error_str = "ERROR: Tree crashed with exception [ " + std::string(ex.what()) + " ]";
                RCLCPP_ERROR(node_->get_logger(),"Tree crashed with exception: %s", ex.what());
                tree_wrapper_.execution_tree_status_ = "CRASHED";
                tree_wrapper_.execution_tree_error_ = ex.what() ;
                PublishExecutionStatus(true, error_str);
                tree_wrapper_.RemoveTree();
            }
        }
        else
        {
           RCLCPP_INFO(node_->get_logger(),"EXEC TERMINATED");
            rclcpp::shutdown();
        }
    }

    void BehaviorTreeNode::PublishExecutionStatus(bool error, std::string error_data)
    {
      TreeStatus status_msg;

        status_msg.time_start = start_execution_time_;
        status_msg.time = node_->get_clock()->now();
        status_msg.uid = tree_uid_;
        status_msg.name = tree_name_;
        status_msg.file = tree_filename_;
        if (!error)
        {
            switch( tree_wrapper_.status_)
            {
                case BT::NodeStatus::FAILURE:
                    status_msg.status  = "FINISHED";
                    status_msg.data = "FAILURE";
                    break;
                case BT::NodeStatus::RUNNING:
                    status_msg.status  = "RUNNING";
                    break;
                case BT::NodeStatus::SUCCESS:
                    status_msg.status  = "FINISHED";
                    status_msg.data = "SUCCESS";
                    break;
                /*case BT::NodeStatus::PAUSED:
                    status_msg.status  = "PAUSED";
                    break;*/
                default:
                    status_msg.status  = "IDLE";
                    break;
            }
        }
        else
        {
            status_msg.status    = "CRASHED";
            status_msg.data      = error_data;
        }
        bt_execution_status_publisher_->publish(status_msg);
    }

    void BehaviorTreeNode::LoadAllPlugins()
    {
        RCLCPP_INFO(node_->get_logger(),"LOADING PLUGINS");
        LoadPluginsFromROS();

        bool import_from_folder = false;
        node_->get_parameter_or("import_from_folder",import_from_folder,false);

        if(import_from_folder)
        {
            RCLCPP_INFO(node_->get_logger(),"LOADING PLUGINS FROM FOLDER");
            std::string plugins_folder;
            if (!node_->get_parameter("plugins_folder",plugins_folder))
            {
                RCLCPP_WARN(node_->get_logger(),"Import from folder option is set, but folder param is missing");
            }
            else
            {
                LoadPluginsFromFolder(plugins_folder);
            }
            RCLCPP_INFO(node_->get_logger(),"LOADING PLUGINS FROM FOLDER OK");
        }
        RCLCPP_INFO(node_->get_logger(),"LOADED PLUGINS");
    }

    void BehaviorTreeNode::LoadPluginsFromROS()
    {
        RCLCPP_INFO(node_->get_logger(),"LOADING PLUGINS FROM ROS");
        using namespace BT_TinyXML2;

        // ros::package::getPlugins returns a pair of strings for each result.
        // The first one is the name of the package that exported the target xml entry (behavior_tree_ros)
        // and the second one is the value of the attribute (plugin)
        std::vector<std::pair<std::string, std::string>> exported_plugins;

        //TODO:
        //ros::package::getPlugins("behavior_tree_ros", "plugin", exported_plugins);

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

                    RCLCPP_INFO(node_->get_logger(),"Loaded plugin %s from ROS plugin", lib_full_path.c_str());
                    plugin_entry = plugin_entry->NextSiblingElement("plugin");
                }
            }
            catch(const std::runtime_error& ex)
            {
              RCLCPP_ERROR(node_->get_logger(),"Error loading plugin %s in path %s: %s.", plugin.first.c_str(), plugin.second.c_str(), ex.what());
            }
        }
        RCLCPP_INFO(node_->get_logger(),"LOADING PLUGINS FROM ROS OK");
    }
    void BehaviorTreeNode::LoadPluginsFromFolder(const std::string& _folder)
    {
        using namespace boost::filesystem;

        if(!exists(_folder))
        {
          RCLCPP_ERROR(node_->get_logger(),"Plugin folder %s does not exist.", _folder.c_str());
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
              RCLCPP_INFO(node_->get_logger(),"Loaded plugin %s from folder %s", plugin_path.filename().string().c_str(), _folder.c_str());
            }
            catch(const std::runtime_error& ex)
            {
              RCLCPP_ERROR(node_->get_logger(),"Cannot load plugin %s from folder %s. Error: %s", entry.path().filename().string().c_str(), _folder.c_str(), ex.what());
            }
        }
    }

    void BehaviorTreeNode::LoadPlugin(const std::string& _plugin_path)
    {
      try
      {
        tree_wrapper_.factory().registerFromPlugin(_plugin_path);
        loaded_plugins_.emplace(_plugin_path);
      }
      catch(const BT::BehaviorTreeException& ex)
      {
        throw std::runtime_error { ex.what() };
      }
    }

  void BehaviorTreeNode::getParameters (rclcpp::Node::SharedPtr nh)
  {

    RCLCPP_INFO(nh->get_logger(),"DECLARING PARAMS");

    std::string bb_init;
    //1. Declare parameters
    nh->declare_parameter("trees_folder", "src/behaviortree_server/behavior_trees");
    nh->declare_parameter("enable_cout_log", true);
    nh->declare_parameter("enable_minitrace_log", true);
    nh->declare_parameter("enable_rostopic_log", true);
    nh->declare_parameter("enable_file_log", true);
    nh->declare_parameter("enable_zmq_log", true);
    nh->declare_parameter("tree_name", "cross_door_name");
    nh->declare_parameter("tree_file", "test_2.xml");
    nh->declare_parameter("tree_uid", 1);
    nh->declare_parameter("tree_debug", true);
    nh->declare_parameter("tree_auto_restart", true);
    nh->declare_parameter("server_port", 1667);
    nh->declare_parameter("publisher_port", 1666);
    nh->declare_parameter("log_folder", "/tmp/");
    nh->declare_parameter("bb_init", "");

    RCLCPP_INFO(nh->get_logger(),"LOADING PARAMS");

    //2. Load Parameters
    trees_folder_ = nh->get_parameter("trees_folder").as_string();
    nh->get_parameter_or("enable_cout_log",enable_cout_log_,false);
    nh->get_parameter_or("enable_minitrace_log",enable_minitrace_log_,false);
    nh->get_parameter_or("enable_rostopic_log",enable_rostopic_log_,false);
    nh->get_parameter_or("enable_file_log",enable_file_log_,false);
    nh->get_parameter_or("enable_zmq_log",enable_zmq_log_,false);
    nh->get_parameter("tree_name").as_string();
    if (!nh->get_parameter("tree_name",tree_name_)){ tree_name_=""; }
    nh->get_parameter("tree_file",tree_filename_);
    tree_filename_ = nh->get_parameter("tree_file").as_string();
   // if (!nh->get_parameter("tree_file",tree_filename_)){ tree_filename_=""; }
    nh->get_parameter_or("tree_uid",tree_uid_,1);
    nh->get_parameter_or("tree_debug",tree_debug_,false);
    nh->get_parameter_or("tree_auto_restart",tree_auto_restart_,false);
    nh->get_parameter_or("server_port",tree_server_port_,1667);
    nh->get_parameter_or("publisher_port",tree_publisher_port_,1666);
    if (!nh->get_parameter("log_folder",log_folder_)){ log_folder_ = "/tmp/"; }
    if (!nh->get_parameter("bb_init",bb_init)){ bb_init = ""; }

    RCLCPP_INFO(nh->get_logger(),"LOADED PARAMS");

    //Build BB_init Vector
    bb_init.erase(std::remove(bb_init.begin(), bb_init.end(),'['), bb_init.end());
    bb_init.erase(std::remove(bb_init.begin(), bb_init.end(),']'), bb_init.end());
    std::stringstream bb_init_stream(bb_init);
    std::string s;
    while (getline(bb_init_stream, s, ',')) {
      s.erase(std::remove(s.begin(), s.end(), '\''), s.end());
      tree_bb_init_.push_back(s);
    }   

    //Init Log Folder
    const char* home = getenv("HOME");
    log_folder_ = log_folder_.front() == '~' ? std::string(home) + log_folder_.substr(1, log_folder_.size() - 1) : log_folder_;

    RCLCPP_INFO(nh->get_logger(),"GET PARAMS DONE");
  }

   void BehaviorTreeNode::InitializeBlackboard(const std::string& abs_file_path, BT::Blackboard::Ptr blackboard_ptr, const bool sync_bb)
    {
        try 
        {
            // ROS_INFO("Initializing BB from YAML file %s", abs_file_path.c_str());
            YAML::Node config = YAML::LoadFile(abs_file_path);
            for(YAML::const_iterator it=config.begin();it!=config.end();++it)
            {
                const std::string& bb_key = it->first.as<std::string>();
                std::string bb_val = it->second.as<std::string>();
                //TODO
                /*
                const BT::Optional<std::string> bbentry_value_inferred_keyvalues = blackboard_ptr->replaceKeysWithStringValues(bb_val, true); // no effect if it has no key
                if(!bbentry_value_inferred_keyvalues)
                {
                    // but will complain if it has a reference to a wrong key
                    RCLCPP_ERROR(node_->get_logger(),"Init. of BB key %s for value %s, value inference did not succeed: %s", 
                        bb_key.c_str(), 
                        bb_val.c_str(),
                        bbentry_value_inferred_keyvalues.error().c_str());
                    continue; // and skip this init
                }
                else
                    bb_val = bbentry_value_inferred_keyvalues.value();

                RCLCPP_INFO(node_->get_logger(),"Init. BB key [\"%s\"] with value \"%s\"", bb_key.c_str(), bb_val.c_str());
                // use the string here and blackboard_ptr->set(...)
                blackboard_ptr->set(bb_key, bb_val, sync_bb);*/
            }
            RCLCPP_INFO(node_->get_logger(),"Initialized BB with %ld entries from YAML file %s", blackboard_ptr->getKeys().size(), abs_file_path.c_str());
        }
        catch(const YAML::Exception& ex) 
        { 
            RCLCPP_ERROR(node_->get_logger(),"Initializing. BB key from file '%s' did not succeed: %s", abs_file_path.c_str(), ex.what());
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


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("behavior_tree_node");

  RCLCPP_INFO(nh->get_logger(),"START");
  auto bt_node = std::make_shared<BehaviorTreeNode>(nh);
  
  rclcpp::Rate rate(30);
  while(rclcpp::ok())
  {
    rclcpp::spin_some(nh);
    //bt_node->Loop();
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}