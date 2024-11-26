#include "behaviortree_node.hpp"
#include <chrono>

namespace BT_SERVER
{
  BehaviorTreeNode::BehaviorTreeNode(const rclcpp::Node::SharedPtr& node) : tree_wrapper_(node), node_(node) 
  {
    //Fetch Tree Parameters
    getParameters(node);

    //Create BT_NODE Tree ROS Services
    RCLCPP_INFO(node_->get_logger(),"CREATING SERVICES");
    get_loaded_plugins_srv_ =node->create_service<GetLoadedPluginsSrv>("/"+tree_name_+"/get_loaded_plugins",std::bind(&BehaviorTreeNode::GetLoadedPluginsServiceCallback,this,_1,_2));
    pause_tree_srv_  = node->create_service<TriggerSrv>("/"+tree_name_+"/pause_tree", std::bind(&BehaviorTreeNode::PauseTreeCallback,this,_1,_2));
    resume_tree_srv_  = node->create_service<TriggerSrv>("/"+tree_name_+"/pause_tree", std::bind(&BehaviorTreeNode::ResumeTreeCallback,this,_1,_2));
    stop_tree_srv_ = node_->create_service<EmptySrv>("/"+tree_name_+"/stop_tree",std::bind(&BehaviorTreeNode::StopTreeCallback,this,_1,_2));
    restart_tree_srv_ = node_->create_service<EmptySrv>("/"+tree_name_+"/restart_tree",std::bind(&BehaviorTreeNode::RestartTreeCallback,this,_1,_2));
    get_tree_status_srv_ = node_->create_service<GetTreeStatusSrv>("/"+tree_name_+"/status_tree",std::bind(&BehaviorTreeNode::StatusTreeCallback,this,_1,_2));
    RCLCPP_INFO(node_->get_logger(),"CREATING SERVICES OK");

    //Init Publishers
    tree_wrapper_.InitializeStatusPublisher();

    if (tree_wrapper_.tree_uid_ >= 0) 
    {
      //Load Plugins
      tree_wrapper_.LoadAllPlugins();

      //Create Blackboard
      tree_wrapper_.InitializeBlackboard();

      //Load Tree
      LoadTree();

      tree_wrapper_.start_execution_time_ = node_->get_clock()->now();
      tree_wrapper_.SetTreeLoaded(true);

      RCLCPP_INFO(node_->get_logger(),"TREE LOADED OK");
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(),"ERROR: UID CAN'T HAVE NEGATIVE VALUE");
        return;
    }

    RCLCPP_INFO(node_->get_logger(),"CREATE PUB & SUB to SYNC values with BT_SERVER");

    // Updates subscriber server side
    sync_bb_sub_ = node_->create_subscription<BBEntry>("behavior_tree_server/broadcast_update", 10, std::bind(&BehaviorTreeNode::SyncBlackboardUpdateCallback, this, _1)) ;
    
    // Updates republisher for all trees (put latch to true atm, because seems a good option that you receive last update from the server)
    sync_bb_pub_ = node_->create_publisher<BBEntry>("/behavior_tree_server/local_update", 10);

    // Publish the initial status (IDLE + no tree loaded).
    tree_wrapper_.UpdatePublishTreeExecutionStatus(BT::NodeStatus::IDLE,false);

    // Publish Paused Status Timer
    check_paused_timer_ = node_->create_wall_timer(std::chrono::milliseconds(loop_rate_), std::bind(&BehaviorTreeNode::CheckPausedCallback,this));
  }

  bool BehaviorTreeNode::LoadTree()
  {
    const auto& full_path = GetFullPath(tree_wrapper_.tree_filename_, trees_folder_);

    //Init error in case of CRASH
    tree_wrapper_.execution_tree_status_ = "ERROR LOADING";
    tree_wrapper_.execution_tree_error_ = "COULD NOT LOAD";

    // If there's a tree being executed, halt and destoy it to execute the new one
    if (tree_wrapper_.IsTreeLoaded()) { tree_wrapper_.RemoveTree(); }

    try
    {
      RCLCPP_INFO(node_->get_logger(),"CREATING TREE FROM FILE %s", full_path.c_str());
      tree_wrapper_.CreateTree(full_path);
      RCLCPP_INFO(node_->get_logger(),"CREATED TREE FROM FILE OK");
    }
    catch(const std::runtime_error& ex)
    {
      RCLCPP_ERROR(node_->get_logger(),"Error loading tree %s: %s", full_path.c_str(), ex.what());

      std::string error_str = "Error loading tree " + full_path + " " + std::string(ex.what());;
      tree_wrapper_.execution_tree_status_ = "ERROR LOADING";
      tree_wrapper_.execution_tree_error_ = ex.what() ;

      tree_wrapper_.PublishExecutionStatus(true,error_str);
      return false;
    }

    //TODO: Set Tree To Debug
    // - Missing TREE->SETDEBUG implementation
    //if (tree_debug_) tree_wrapper_.tree_.setDebug();

    //TODO: Send Sync Blackboard updates
    // - Missing TREE->GETKEYSVALUETOSYNC implementation
    /*sendBlackboardUpdates(tree_wrapper_.getKeysValueToSync()); // send updates  */
    getBlackboardUpdates(); // blocking call to update bb with missing values that need to be retrieved from server

    //Init Loggers
    tree_wrapper_.InitializeLoggers();

    tree_wrapper_.execution_tree_status_ = "IDLE";
    tree_wrapper_.execution_tree_error_ = "";
    tree_wrapper_.UpdatePublishTreeExecutionStatus(BT::NodeStatus::IDLE, false);

    RCLCPP_INFO(node_->get_logger(),"Loaded srv tree %s counting of %ld nodes", full_path.c_str(), tree_wrapper_.TreeNodesCount());
    return true;
  }

  void BehaviorTreeNode::CheckPausedCallback()
  {
    //TODO: Check if Tree is Paused
    // - Missing BT::NodeStatus::PAUSED definition

    /*if(tree_wrapper_.IsTreeLoaded())
    {
        if(tree_wrapper_.IsTreePaused() && tree_wrapper_.GetTreeStatus() != BT::NodeStatus::PAUSED)
        {
            tree_wrapper_.UpdatePublishTreeExecutionStatus(BT::NodeStatus::PAUSED);
        } 
    }*/
  }

  void BehaviorTreeNode::SyncBlackboardUpdateCallback(const BBEntry::SharedPtr _topic_msg)
  {   
    // Forward received update to tree
    if(tree_wrapper_.IsTreeLoaded()) tree_wrapper_.SyncBlackboardUpdateCallback(*_topic_msg);
  }

  //TODO: Notify Changed SYNC PORT VALUES
  // - Missing BLACKBOARD::SerializedEntriesMap definition
  /*void BehaviorTreeNode::sendBlackboardUpdates(const BT::Blackboard::SerializedEntriesMap& entries_map)
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

  void BehaviorTreeNode::getBlackboardUpdates(const bool just_empty_values)
  {
    //TODO:
    // - GetSyncKeys Missing
   /* 
    rclcpp::Client<GetBBValues>::SharedPtr client = node_->create_client<GetBBValues>("/behavior_tree_server/get_sync_bb_values"); 
    auto request = std::make_shared<GetBBValues::Request>();

    const std::unordered_set<std::string> keys = getSyncKeys(just_empty_values);
    request->keys = std::vector<std::string>(keys.begin(), keys.end());

    while (!client->wait_for_service()) 
    {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      tree_wrapper_.SyncBlackboardUpdateCallback(result.get()->entries);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR: Failed to call service get_sync_bb_values");
    }*/
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

      if(tree_wrapper_.AreLoggersInitialized() && !tree_wrapper_.HasExecutionTerminated()) { 
          try
          {
              RCLCPP_INFO(node_->get_logger(),"TICK ONCE");
              const auto tree_status = tree_wrapper_.tree_.tickOnce();
              RCLCPP_INFO(node_->get_logger(),"TICK ONCE OK");

              //TODO:
              //sendBlackboardUpdates(tree_wrapper_.getKeysValueToSync());
              
              // Publish the updated status if there have been changes.
              if(tree_status != tree_wrapper_.GetTreeStatus())
              {
                  tree_wrapper_.UpdatePublishTreeExecutionStatus(tree_status);
              }

              // Publish the updated status
              if(tree_status == BT::NodeStatus::FAILURE)
              {
                  tree_wrapper_.ResetTree();
                  tree_wrapper_.SetExecuted(!tree_auto_restart_); // if auto restart is false, set executed to true to stop the tick, otherwise will restart the tick from the beginning
              }
              else if(tree_status == BT::NodeStatus::SUCCESS)
              {
                  tree_wrapper_.ResetTree();
                  tree_wrapper_.SetExecuted(!tree_auto_restart_); // if auto restart is false, set executed to true to stop the tick, otherwise will restart the tick from the beginning
              }
          }
          catch(const BT::BehaviorTreeException& ex)
          {
              std::string error_str = "ERROR: Tree crashed with exception [ " + std::string(ex.what()) + " ]";
              RCLCPP_ERROR(node_->get_logger(),"Tree crashed with exception: %s", ex.what());
              tree_wrapper_.execution_tree_status_ = "CRASHED";
              tree_wrapper_.execution_tree_error_ = ex.what() ;
              tree_wrapper_.PublishExecutionStatus(true, error_str);
              RemoveTree();
          }
      }
      else
      {
          RCLCPP_INFO(node_->get_logger(),"EXEC TERMINATED");
          rclcpp::shutdown();
      }
  }


  bool BehaviorTreeNode::GetLoadedPluginsServiceCallback(const std::shared_ptr<GetLoadedPluginsSrv::Request> _request, std::shared_ptr<GetLoadedPluginsSrv::Response> _response)
  {
    _response->plugins.assign(tree_wrapper_.loaded_plugins_.cbegin(), tree_wrapper_.loaded_plugins_.cend());
    return true;
  }

  void BehaviorTreeNode::RemoveTree()
  {
      tree_wrapper_.RemoveTree();

      // Update and publish the status here too
      // (neeed to cover the case where users manually
      // stop a tree by calling the stop_tree service).
      tree_wrapper_.UpdatePublishTreeExecutionStatus(BT::NodeStatus::IDLE, false);
  }

  bool BehaviorTreeNode::StopTree()
  {
      if(tree_wrapper_.IsTreeLoaded())
      {
          RemoveTree();
          RCLCPP_INFO(node_->get_logger(),"Tree stopped");
      }
      return !tree_wrapper_.IsTreeLoaded();
  }

  bool BehaviorTreeNode::StopTreeCallback(const std::shared_ptr<EmptySrv::Request> _request, std::shared_ptr<EmptySrv::Response> _response)
  {
    tree_wrapper_.execution_tree_status_ = "FINISHED";
    tree_wrapper_.execution_tree_error_ = "Canceled by StopTree Service";
    tree_wrapper_.UpdatePublishTreeExecutionStatus(BT::NodeStatus::IDLE, false);
    tree_wrapper_.ResetTree();
    return StopTree();
  }
  bool BehaviorTreeNode::PauseTreeCallback(const std::shared_ptr<TriggerSrv::Request> _request, std::shared_ptr<TriggerSrv::Response> _response)
  {
    //TODO:
    //  _response = tree_wrapper_.tree().PauseResume(true);
      return true;
  }
  bool BehaviorTreeNode::ResumeTreeCallback(const std::shared_ptr<TriggerSrv::Request> _request, std::shared_ptr<TriggerSrv::Response> _response)
  {
    //TODO:
    //  _response = tree_wrapper_.tree().PauseResume(false);
      return true;
  }
  bool BehaviorTreeNode::RestartTreeCallback(const std::shared_ptr<EmptySrv::Request> _request, std::shared_ptr<EmptySrv::Response> _response)
  {
    if (tree_wrapper_.IsTreeLoaded())
    {
      tree_wrapper_.SetExecuted(false);
      return true;
    }
    return false;
  }
  bool BehaviorTreeNode::StatusTreeCallback(const std::shared_ptr<GetTreeStatusSrv::Request> _request, std::shared_ptr<GetTreeStatusSrv::Response> _response)
  {
    if (tree_wrapper_.IsTreeLoaded())
    {
      _response->status = tree_wrapper_.buildTreeExecutionStatus();
      return true;
    }
    _response->status.details = "NOT LOADED";
    return false;
  }

  void BehaviorTreeNode::getParameters (rclcpp::Node::SharedPtr nh)
  {

    RCLCPP_INFO(nh->get_logger(),"DECLARING PARAMS");

    // Declare parameters
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
    nh->declare_parameter("plugins_dir","[]");

    RCLCPP_INFO(nh->get_logger(),"LOADING PARAMS");
      
    // Load Parameters
    trees_folder_ = nh->get_parameter("trees_folder").as_string();

    tree_name_= nh->get_parameter("tree_name").as_string();

    tree_debug_ = nh->get_parameter("tree_debug").as_bool();
    tree_auto_restart_ = nh->get_parameter("tree_auto_restart").as_bool();

    tree_wrapper_.tree_name_ = tree_name_;
    tree_wrapper_.tree_uid_ = nh->get_parameter("tree_uid").as_int();
    tree_wrapper_.tree_filename_ = nh->get_parameter("tree_file").as_string();
    tree_wrapper_.enable_cout_log_ = nh->get_parameter("enable_cout_log").as_bool();
    tree_wrapper_.enable_minitrace_log_ = nh->get_parameter("enable_minitrace_log").as_bool();
    tree_wrapper_.enable_rostopic_log_ = nh->get_parameter("enable_rostopic_log").as_bool();
    tree_wrapper_.enable_file_log_ = nh->get_parameter("enable_file_log").as_bool();
    tree_wrapper_.enable_zmq_log_ = nh->get_parameter("enable_zmq_log").as_bool();
    tree_wrapper_.log_folder_ = nh->get_parameter("log_folder").as_string();
    tree_wrapper_.tree_server_port_ = nh->get_parameter("server_port").as_int();
    tree_wrapper_.tree_publisher_port_ = nh->get_parameter("publisher_port").as_int();
    tree_wrapper_.params_.groot2_port = tree_wrapper_.tree_server_port_;
    tree_wrapper_.ros_plugin_directories_ = node_->get_parameter("plugins_dir").as_string_array();

    //Build BB_init Vector (OLD WAY)
    /*std::string bb_init;
    bb_init = nh->get_parameter("bb_init").as_string();
    bb_init.erase(std::remove(bb_init.begin(), bb_init.end(),'['), bb_init.end());
    bb_init.erase(std::remove(bb_init.begin(), bb_init.end(),']'), bb_init.end());
    std::stringstream bb_init_stream(bb_init);
    std::string s;
    while (getline(bb_init_stream, s, ',')) {
      s.erase(std::remove(s.begin(), s.end(), '\''), s.end());
      tree_wrapper_.tree_bb_init_.push_back(s);
    }*/

    //Build BB_init Vector (New WAY TO TEST)
    tree_wrapper_.tree_bb_init_ = node_->get_parameter("bb_init").as_string_array();

    RCLCPP_INFO(nh->get_logger(),"LOADED PARAMS OK");
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("behavior_tree_node");

  RCLCPP_INFO(nh->get_logger(),"START");
  auto bt_node = std::make_shared<BT_SERVER::BehaviorTreeNode>(nh);
  
  rclcpp::Rate rate(bt_node->loop_rate_);
  while(rclcpp::ok())
  {
    rclcpp::spin_some(nh);
    bt_node->Loop();
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}