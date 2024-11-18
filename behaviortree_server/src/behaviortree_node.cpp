#include "behaviortree_node.hpp"

BehaviorTreeNode::BehaviorTreeNode(const rclcpp::Node::SharedPtr& node) : tree_wrapper_(node), node_(node) 
{
  //Fetch Tree Parameters
  getParameters(node);

  //Create BT_NODE Tree ROS Services
  RCLCPP_INFO(node_->get_logger(),"CREATING SERVICES");
  get_loaded_plugins_srv_ =node->create_service<GetLoadedPluginsSrv>("/"+tree_name_+"/get_loaded_plugins",std::bind(&BehaviorTreeNode::GetLoadedPluginsService,this,_1,_2));
  stop_tree_srv_ = node_->create_service<EmptySrv>("/"+tree_name_+"/stop_tree",std::bind(&BehaviorTreeNode::StopTree,this,_1,_2));
  restart_tree_srv_ = node_->create_service<EmptySrv>("/"+tree_name_+"/restart_tree",std::bind(&BehaviorTreeNode::RestartTree,this,_1,_2));
  get_tree_status_srv_ = node_->create_service<GetTreeStatusSrv>("/"+tree_name_+"/status_tree",std::bind(&BehaviorTreeNode::StatusTree,this,_1,_2));
  RCLCPP_INFO(node_->get_logger(),"CREATING SERVICES OK");

  //Init Loggers
  InitializeLoggers();

  if (tree_uid_ >= 0) 
  {
    //Load Plugins
    tree_wrapper_.LoadAllPlugins();

    //Create Blackboard
    tree_wrapper_.InitializeBlackboard();

    //Create Tree
    const auto& full_path = GetFullPath(tree_filename_, trees_folder_);
    RCLCPP_INFO(node_->get_logger(),"CREATING TREE FROM FILE %s", full_path.c_str());
    tree_wrapper_.CreateTree(full_path);
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
      RCLCPP_ERROR(node_->get_logger(),"ERROR: UID CAN'T HAVE NEGATIVE VALUE");
      return;
  }

  RCLCPP_INFO(node_->get_logger(),"CREATE PUB & SUB to SYNC values with BT_SERVER");

  //Updates subscriber server side
  sync_bb_sub_ = node_->create_subscription<BBEntry>("behavior_tree_server/broadcast_update", 10, std::bind(&BehaviorTreeNode::SyncBlackboardUpdateCallback, this, _1)) ;
  //Updates republisher for all trees (put latch to true atm, because seems a good option that you receive last update from the server)
  sync_bb_pub_ = node_->create_publisher<BBEntry>("/behavior_tree_server/local_update", 10);

  RCLCPP_INFO(node_->get_logger(),"CREATE BT_SERVER PUB & SUB OK");
}

void BehaviorTreeNode::InitializeLoggers()
{
  //Create Loggers
  if (enable_file_log_)
  {
    const char* home = getenv("HOME");
    log_folder_ = log_folder_.front() == '~' ? std::string(home) + log_folder_.substr(1, log_folder_.size() - 1) : log_folder_;
    //TODO: ENABLE LOG FOLDER
  }
  if (enable_minitrace_log_)
  {
    //TODO:
  }
  if (enable_cout_log_)
  {
    //TODO:
  }
  if(enable_rostopic_log_)
  {
    RCLCPP_INFO(node_->get_logger(),"INIT ROSTOPIC LOGS");
    InitializeStatusPublisher(tree_name_);
    RCLCPP_INFO(node_->get_logger(),"INIT ROSTOPIC LOGS OK");
  }

  if (enable_zmq_log_) 
  {
    //TODO:
    tree_wrapper_.InitGrootPublisher();
  }

  loggers_init_ = true;
}

bool BehaviorTreeNode::GetLoadedPluginsService(const std::shared_ptr<GetLoadedPluginsSrv::Request> _request, std::shared_ptr<GetLoadedPluginsSrv::Response> _response)
{
  //TODO: loaded_plugins_ not filled!
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

/* TODO:
void sendBlackboardUpdates(const BT::Blackboard::SerializedEntriesMap& entries_map)
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

/* TODO:
void getBlackboardUpdates(const bool just_empty_values)
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

/* TODO:
std::unordered_set<std::string> getSyncKeys(const bool just_empty_values)
{
  return tree_wrapper_.globalBlackboard()->getSyncKeys(just_empty_values);
}*/

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

            //TODO:
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
  enable_cout_log_ = nh->get_parameter("enable_cout_log").as_bool();
  enable_minitrace_log_ = nh->get_parameter("enable_minitrace_log").as_bool();
  enable_rostopic_log_ = nh->get_parameter("enable_rostopic_log").as_bool();
  enable_file_log_ = nh->get_parameter("enable_file_log").as_bool();
  enable_zmq_log_ = nh->get_parameter("enable_zmq_log").as_bool();
  tree_name_= nh->get_parameter("tree_name").as_string();
  tree_filename_ = nh->get_parameter("tree_file").as_string();
  tree_uid_ = nh->get_parameter("tree_uid").as_int();
  tree_debug_ = nh->get_parameter("tree_debug").as_bool();
  tree_auto_restart_ = nh->get_parameter("tree_auto_restart").as_bool();
  log_folder_ = nh->get_parameter("log_folder").as_string();

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
    bt_node->Loop();
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}