#include "rclcpp/rclcpp.hpp"
#include <behaviortree_ros2/tree_execution_server.hpp>

#include "std_srvs/srv/empty.hpp"
#include "behaviortree_server_interfaces/msg/bb_entry.hpp"
#include "behaviortree_server_interfaces/srv/get_loaded_plugins.hpp"
#include "behaviortree_server_interfaces/srv/get_tree_status.hpp"
#include "behaviortree_server_interfaces/srv/get_bb_values.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using BBEntry = behaviortree_server_interfaces::msg::BBEntry;
using GetLoadedPluginsSrv = behaviortree_server_interfaces::srv::GetLoadedPlugins;
using GetTreeStatusSrv = behaviortree_server_interfaces::srv::GetTreeStatus;
using GetBBValues = behaviortree_server_interfaces::srv::GetBBValues;

using EmptySrv = std_srvs::srv::Empty;

class BehaviorTreeNode : public rclcpp::Node
{
  public:
    BehaviorTreeNode(const rclcpp::NodeOptions& options) : Node("BehaviorTreeNode") , tree_loader_service_(options)
    {
          
      /*  
      LoadAllPlugins();
      */

      //Create Tree Services
      get_loaded_plugins_srv_ = this->create_service<GetLoadedPluginsSrv>("/"+tree_loader_service_.treeName()+"/get_loaded_plugins",std::bind(&BehaviorTreeNode::GetLoadedPluginsService,this,_1,_2));
      stop_tree_srv_ = this->create_service<EmptySrv>("/"+tree_loader_service_.treeName()+"/stop_tree",std::bind(&BehaviorTreeNode::StopTree,this,_1,_2));
      restart_tree_srv_ = this->create_service<EmptySrv>("/"+tree_loader_service_.treeName()+"/restart_tree",std::bind(&BehaviorTreeNode::RestartTree,this,_1,_2));
      get_tree_status_srv_ = this->create_service<GetTreeStatusSrv>("/"+tree_loader_service_.treeName()+"/status_tree",std::bind(&BehaviorTreeNode::StatusTree,this,_1,_2));
       
      /*
      if(enable_rostopic_log_)
      {
          service_tree_.InitializeStatusPublisher(public_node_handle_,stree_loader_service_.treeName);
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
              RCLCPP_ERROR(this->get_logger(),"ERROR: FAILED TO LOAD THE TREE");
          }
      }
      else
      {
          RCLCPP_ERROR(this->get_logger(),"ERROR: UID, SERVER_PORT and PUBLISHER_PORT CAN'T HAVE NEGATIVE VALUES");
      }
      */

      //Updates subscriber server side
      sync_bb_sub_ = this->create_subscription<BBEntry>("behavior_tree_server/broadcast_update", 10, std::bind(&BehaviorTreeNode::SyncBlackboardUpdateCallback, this, _1)) ;
      //Updates republisher for all trees (put latch to true atm, because seems a good option that you receive last update from the server)
      sync_bb_pub_ = this->create_publisher<BBEntry>("/behavior_tree_server/local_update", 10);
    }

  private:
    bool GetLoadedPluginsService(const std::shared_ptr<GetLoadedPluginsSrv::Request> _request, std::shared_ptr<GetLoadedPluginsSrv::Response> _response)
    {
      _response->plugins.assign(loaded_plugins_.cbegin(), loaded_plugins_.cend());
      return true;
    }
    bool StopTree(const std::shared_ptr<EmptySrv::Request> _request, std::shared_ptr<EmptySrv::Response> _response)
    {
      //TODO
      return true;
    }
    bool RestartTree(const std::shared_ptr<EmptySrv::Request> _request, std::shared_ptr<EmptySrv::Response> _response)
    {
      //TODO
      return true;
    }
    bool StatusTree(const std::shared_ptr<GetTreeStatusSrv::Request> _request, std::shared_ptr<GetTreeStatusSrv::Response> _response)
    {
      //TODO
      return true;
    }
    
    void SyncBlackboardUpdateCallback(const BBEntry::SharedPtr _topic_msg)
    {   
      //TODO
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
        rclcpp::Client<GetBBValues>::SharedPtr client = this->create_client<GetBBValues>("/behavior_tree_server/get_sync_bb_values"); 
        GetBBValues::Request request;
        GetBBValues::Response response;
        const std::unordered_set<std::string> keys = getSyncKeys(just_empty_values);
        request.keys = std::vector<std::string>(keys.begin(), keys.end());
        if(client.call(request, response))
        {
            SyncBlackboardUpdateCallback(response.entries, &tree_loader_service_.factory());
        }
    }*/
    /* std::unordered_set<std::string> getSyncKeys(const bool just_empty_values)
    {
      return tree_loader_service_.globalBlackboard()->getSyncKeys(just_empty_values);
    }*/

    BT::TreeExecutionServer tree_loader_service_;
    std::string trees_folder_;
    std::string tree_name_;
    std::string tree_filename_;
    std::vector<std::string> tree_bb_init_ {};
    int tree_uid_;
    bool tree_debug_;
    bool tree_auto_restart_;
    int tree_server_port_;
    int tree_publisher_port_;
    std::string log_folder_;
    bool enable_cout_log_;
    bool enable_minitrace_log_;
    bool enable_rostopic_log_;
    bool enable_file_log_;
    bool enable_zmq_log_;

    std::set<std::string> loaded_plugins_;

    //Services
    rclcpp::Service<GetLoadedPluginsSrv>::SharedPtr get_loaded_plugins_srv_;
    rclcpp::Service<EmptySrv>::SharedPtr stop_tree_srv_;
    rclcpp::Service<EmptySrv>::SharedPtr restart_tree_srv_;
    rclcpp::Service<GetTreeStatusSrv>::SharedPtr get_tree_status_srv_;

    //Subscribers
    rclcpp::Subscription<BBEntry>::SharedPtr sync_bb_sub_;
    
    //Publishers
    rclcpp::Publisher<BBEntry>::SharedPtr sync_bb_pub_;
};

/*rclpp:NodeOptions getParameters (rclcpp::Node::SharedPtr nh)
{
  rclcpp::NodeOptions options;

  std::string bb_init;
  //LOAD TREE Parameters
  trees_folder_ = nh->get_parameter("trees_folder").as_string();
  nh->get_parameter_or("enable_cout_log",enable_cout_log_,false);
  nh->get_parameter_or("enable_minitrace_log",enable_minitrace_log_,false);
  nh->get_parameter_or("enable_rostopic_log",enable_rostopic_log_,false);
  nh->get_parameter_or("enable_file_log",enable_file_log_,false);
  nh->get_parameter_or("enable_zmq_log",enable_zmq_log_,false);
  nh->get_parameter("tree_name").as_string();
  if (!nh->get_parameter("tree_name",tree_name_)){ tree_name_=""; }
  if (!nh->get_parameter("tree_file",tree_filename_)){ tree_filename_=""; }
  nh->get_parameter_or("tree_uid",tree_uid_,1);
  nh->get_parameter_or("tree_debug",tree_debug_,false);
  nh->get_parameter_or("tree_auto_restart",tree_auto_restart_,false);
  nh->get_parameter_or("server_port",tree_server_port_,1667);
  nh->get_parameter_or("publisher_port",tree_publisher_port_,1666);
  if (!nh->get_parameter("log_folder",log_folder_)){ log_folder_ = "/tmp/"; }
  if (!nh->get_parameter("log_folder",bb_init)){ bb_init = ""; }

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

  return options;
}*/

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("behavior_tree_node");
  
  //rclcpp::NodeOptions options = getParameters(nh);
  //auto bt_node = std::make_shared<BehaviorTreeNode>(options);
  
  while(rclcpp::ok())
  {

  }
  return 0;
}