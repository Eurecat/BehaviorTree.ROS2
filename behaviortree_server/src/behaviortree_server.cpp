#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/empty.hpp"
#include "behaviortree_server_interfaces/msg/bb_entry.hpp"
#include "behaviortree_server_interfaces/msg/tree_execution_status.hpp"

#include "behaviortree_server_interfaces/srv/load_tree.hpp"
#include "behaviortree_server_interfaces/srv/get_tree_status_by_id.hpp"
#include "behaviortree_server_interfaces/srv/get_tree_status.hpp"
#include "behaviortree_server_interfaces/srv/get_bb_values.hpp"
#include "behaviortree_server_interfaces/srv/tree_request.hpp"
#include "behaviortree_server_interfaces/srv/get_all_trees_status.hpp"

#include "behaviortree_cpp/blackboard.h"
#include "yaml-cpp/yaml.h"
#include <behaviortree_cpp/bt_factory.h>

#include "ros2_launch_manager.hpp"
#include "utils.hpp"

#include <chrono>
using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

using BBEntry = behaviortree_server_interfaces::msg::BBEntry;
using TreeStatus = behaviortree_server_interfaces::msg::TreeExecutionStatus;
using LoadTreeSrv = behaviortree_server_interfaces::srv::LoadTree;
using TreeRequestSrv = behaviortree_server_interfaces::srv::TreeRequest;
using GetBBValuesSrv = behaviortree_server_interfaces::srv::GetBBValues;
using GetTreeStatusSrv = behaviortree_server_interfaces::srv::GetTreeStatusByID;
using GetAllTreeStatusSrv = behaviortree_server_interfaces::srv::GetAllTreesStatus;
using EmptySrv = std_srvs::srv::Empty;

class TreeProcessInfo
{
  public:
    TreeProcessInfo(std::string in_tree_name, pid_t in_pid)
    {
        tree_name = in_tree_name;
        pid = in_pid;
    };
    pid_t pid;
    std::string tree_name;
    TreeStatus tree_status;
    rclcpp::Subscription<TreeStatus>::SharedPtr status_subscriber;
};

class BehaviorTreeServer
{
  public:
    BehaviorTreeServer(const rclcpp::Node::SharedPtr& node) : node_(node) 
    {
        //Create Sync_BB and Init
        sync_blackboard_ptr_ = BT::Blackboard::create();
        std::string sync_bb_init_file;
        if (!node_->get_parameter("sync_bb_init",sync_bb_init_file)){ sync_bb_init_file = ""; }
        if(sync_bb_init_file.length() > 0) { InitializeBlackboard(sync_bb_init_file, sync_blackboard_ptr_, true); }

        //Create Services:
        load_tree_srv_ = node_->create_service<LoadTreeSrv>("behavior_tree_server/load_tree",std::bind(&BehaviorTreeServer::LoadTree,this,_1,_2));
        stop_tree_srv_ = node_->create_service<TreeRequestSrv>("behavior_tree_server/stop_tree",std::bind(&BehaviorTreeServer::StopTree,this,_1,_2));
        kill_tree_srv_ = node_->create_service<TreeRequestSrv>("behavior_tree_server/kill_tree",std::bind(&BehaviorTreeServer::KillTree,this,_1,_2));
        kill_all_trees_srv_ = node_->create_service<EmptySrv>("behavior_tree_server/kill_all_trees",std::bind(&BehaviorTreeServer::KillAllTrees,this,_1,_2));
        restart_tree_srv_  = node_->create_service<TreeRequestSrv>("behavior_tree_server/restart_tree",std::bind(&BehaviorTreeServer::RestartTree,this,_1,_2));
        get_sync_bb_values_srv_ = node_->create_service<GetBBValuesSrv>("behavior_tree_server/get_sync_bb_values",std::bind(&BehaviorTreeServer::GetSyncBBValues,this,_1,_2));
        get_tree_status_srv_ = node_->create_service<GetTreeStatusSrv>("behavior_tree_server/get_tree_status",std::bind(&BehaviorTreeServer::GetTreeStatus,this,_1,_2));
        get_all_trees_status_srv_ = node_->create_service<GetAllTreeStatusSrv>("behavior_tree_server/get_all_trees_status",std::bind(&BehaviorTreeServer::GetAllTreeStatus,this,_1,_2));
        
        //Updates subscriber server side
        sync_bb_sub_ = node_->create_subscription<BBEntry>("behavior_tree_server/local_update", 10, std::bind(&BehaviorTreeServer::sync_bb_callback, this, _1)) ;
        
        //Updates republisher for all trees (put latch to true atm, because seems a good option that you receive last update from the server)
        sync_bb_pub_ = node_->create_publisher<BBEntry>("behavior_tree_server/broadcast_update", 10);
    }

  private:

    void sync_bb_callback(const BBEntry::SharedPtr msg) const
    {
    /*  RCLCPP_INFO(this->get_logger(), "KEY: '%s' VALUE: '%s'", msg->key.c_str(), msg->value.c_str());

      //TODO 1: SYNC_BB CALLBACK
      
      const bool void_type = (msg->type == BT::demangle(typeid(void))); // source tree does not know the type of the value
      //retrieve string converter functor
      const BT::StringConverter* string_converter_ptr = void_type? nullptr : sync_blackboard_ptr_->getEntry(msg->key).get()->string_converter;
      if(!void_type && string_converter_ptr == nullptr)
      {
          RCLCPP_ERROR(this->get_logger(),"[BTServer] Entry in Sync. BB for key [%s] has type [%s], but no string converter can be found for this type",  msg->key.c_str(),  msg->type.c_str());
          return;
      }
      const BT::Blackboard::Entry* entry_check_ptr = sync_blackboard_ptr_->getEntry(msg->key).get();
        if(entry_check_ptr == nullptr || ( sync_blackboard_ptr_->entryInfo(msg->key)==nullptr && !void_type))
        {
            // Entry not present in the BB -> First insert
            BT::Optional<BT::PortInfo> port_info_opt = sync_blackboard_ptr_->entryInfo(msg->key);
            if(!port_info_opt.has_value())
            {
                RCLCPP_ERROR(this->get_logger(),"[BTServer] Entry in Sync. BB for key [%s] has type [%s], but it is an unknown type and therefore cannot be treated", msg->key.c_str(),  msg->type.c_str());
                return; // type unknown
            }
            
            // Set empty entry with type info
            sync_blackboard_ptr_->createEntry(msg->key, port_info_opt.value());
            RCLCPP_INFO(this->get_logger(),"[BTServer] Entry in Sync. BB for key [%s] updated with type [%s]", msg->key.c_str(), BT::demangle(port_info_opt.value().type()).c_str());
        }

        //retrieve current entry in bt server bb
        auto entry_ptr = sync_blackboard_ptr_->getEntry(msg->key);
        if(entry_ptr)
        {
            //entry already present in the bb -> UPDATE
            
            if(!void_type && msg->type != sync_blackboard_ptr_->entryInfo(msg->key)->type().name())
            {
                RCLCPP_ERROR(this->get_logger(),"[BTServer] Entry in Sync. BB for key [%s] has type [%s], but receiving requests for update with type [%s]", msg->key.c_str(), sync_blackboard_ptr_->entryInfo(msg->key).type().name(), msg->type.c_str());
                return; // type inconsistencies, don't update
            }
            
            try
            {
                if(!void_type)
                {
                    // convert from string new value
                    BT::Any new_any_value = (*string_converter_ptr)(msg->value);

                    // update it into the sync BB
                    sync_blackboard_ptr_->set(msg->key, std::move(new_any_value), true);
                }
                else
                    sync_blackboard_ptr_->set(msg->key, msg->value, true);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(),"[BTServer] Entry in Sync. Fail to update value in BB for key [%s]: %s", msg->key.c_str(), e.what());
                return;
            }

            // if(update_successful)
            {
                BBEntry upd_msg;
                upd_msg.key = msg->key;
                upd_msg.type = sync_blackboard_ptr_->getEntry(msg->key)->info.type().name() ;
                upd_msg.value = msg->value;
                this->sync_bb_pub_->publish(upd_msg);
            }
        }
        */
    }

   bool LoadTree(const std::shared_ptr<LoadTreeSrv::Request> req, std::shared_ptr<LoadTreeSrv::Response> res)
    {
      RCLCPP_INFO(node_->get_logger(), "TREE FILE: '%s' ", req->tree_file.c_str());
 
      //1. Extract Tree Name
      std::string tree_name;
      std::string tree_filename_tmp = req->tree_file;
      std::size_t found1 = tree_filename_tmp.find_last_of("/");
      std::size_t found2 = tree_filename_tmp.find(".xml");
      if((found1 != std::string::npos) && (found2 != std::string::npos))
      {
        tree_name = tree_filename_tmp.substr ( (found1+1) , (tree_filename_tmp.size()-5-found1) );
      }
      else if(found2 != std::string::npos)
      {
        tree_name = tree_filename_tmp.substr (0,(tree_filename_tmp.size()-5));
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(),"TREE FILE NOT VALID %s",tree_filename_tmp.c_str());
        return false;
      }

      //2. Remove invalid characters on the TreeName
      const std::string characters_allowed = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890/_";
      auto new_end = std::remove_if(tree_name.begin(), tree_name.end(),
                                      [characters_allowed](std::string::value_type c)
                                      { return characters_allowed.find(c) == std::string::npos; });
      tree_name.erase(new_end, tree_name.end());

      //3. Force Unique Tree Name
      std::string tree_name_tmp = tree_name;
      int cnt = 1;
      bool tree_name_found = true;
      while (tree_name_found)
      {
          tree_name_found = false;
          for (auto tree_info : uids_to_tree_info)
          {
              if (tree_info.second.tree_name == tree_name_tmp)
              {
                  //Add number to differentiate names
                  tree_name_found = true;
                  tree_name_tmp = tree_name + std::to_string(cnt);
                  cnt++;
              }
          }
      }
      tree_name = tree_name_tmp;

      RCLCPP_INFO(node_->get_logger(), "LOADING TREE: %s",tree_name.c_str());

      trees_UID++;
      std::string param_name = "tree_name:="+tree_name;
      std::string param_file = "tree_file:="+req->tree_file;
      std::string param_uid = "tree_uid:="+std::to_string(trees_UID);
      std::string param_auto_restart = "tree_auto_restart:="+BoolToString(req->auto_restart);
      std::string param_debug ="tree_debug:="+BoolToString(req->debug);

      std::string param_bb_init = "tree_bb_init:=\\'";
      if (req->bb_init_files.size()> 0)
      {
          param_bb_init += "[";
          for (long unsigned int i = 0; i < req->bb_init_files.size(); i++)
          {
              param_bb_init += req->bb_init_files[i];
              if (i != (req->bb_init_files.size()-1))
                    param_bb_init += ",";
          }
          param_bb_init += "]\\'";
      }
      else
          param_bb_init += "[]\\'";

      //Set default port IDs
      int server_port = 1667;
      int publisher_port = 1666;

      //Get port parameters
      if (req->server_port > 0)
          server_port = req->server_port;
      if (req->publisher_port > 0)
          publisher_port = req->publisher_port;

      std::string param_server_port = "server_port:="+std::to_string(server_port);  
      std::string param_pub_port ="publisher_port:="+std::to_string(publisher_port);
        
      pid_t pid;

      //CREATE ROS2 RUN MANAGER
      const char* package_name = "behaviortree_server"; 
      const char* executable_name = "behaviortree_node";
      try {
        pid = ros2_launch_manager.start(node_,
              package_name,
              executable_name,
              "--ros-args",
              "-p", param_name.c_str(),
              "-p", param_file.c_str(),
              "-p", param_uid.c_str(),
              "-p", param_debug.c_str(),
              "-p", param_auto_restart.c_str(),
              "-p", param_bb_init.c_str(),
              "-p", param_server_port.c_str(),
              "-p", param_pub_port.c_str()
              );
      }
      catch (std::exception const &exception) {
        RCLCPP_WARN(node_->get_logger(),"%s", exception.what());
        return false;
      }

      TreeProcessInfo new_process_info {tree_name,pid};
      std::string topic_name = "/"+tree_name+"/execution_status";
      new_process_info.status_subscriber =  node_->create_subscription<TreeStatus>(topic_name, 10, std::bind(&BehaviorTreeServer::TreeStatusTopicCB, this, _1));
      uids_to_tree_info.emplace(trees_UID,new_process_info);
      res->tree_uid = trees_UID;
     
      RCLCPP_INFO(node_->get_logger(),"LOADING %s OK", tree_name.c_str());

      return true;
    }
    bool StopTree(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res)
    {
      RCLCPP_INFO(node_->get_logger(), "TREE UID: '%u' ", req->tree_uid);
      if(uids_to_tree_info.find(req->tree_uid) != uids_to_tree_info.end())
      {
          TreeProcessInfo tree_info = uids_to_tree_info.at(req->tree_uid);
          return RosServiceStopCall(tree_info.tree_name);
      }
      return false;
    }
    bool KillTree(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res)
    {
      RCLCPP_INFO(node_->get_logger(), "TREE UID: '%u' ", req->tree_uid);
      if (StopTree(req,res))
        {
            if(uids_to_tree_info.find(req->tree_uid) != uids_to_tree_info.end())
            {
                TreeProcessInfo tree_info = uids_to_tree_info.at(req->tree_uid);
                std::string command = "ros2 lifecycle set /"+tree_info.tree_name+"/behavior_tree_ros_node"+" shutdown; kill -9 "+std::to_string(tree_info.pid);
                int result = system(command.c_str());
                if (result != -1)
                {
                    return true;
                }
                RCLCPP_ERROR(node_->get_logger(), "Failed to kill node");
                return false;
            }
        }
        return false;
    }
    bool KillAllTrees(const std::shared_ptr<EmptySrv::Request> req, std::shared_ptr<EmptySrv::Response> res)
    {
      RCLCPP_INFO(node_->get_logger(), "KILL ALL TREES");
      for (auto tree_info : uids_to_tree_info)
      {
          std::shared_ptr<TreeRequestSrv::Request> request;
          std::shared_ptr<TreeRequestSrv::Response> response;
          request->tree_uid  = tree_info.first;
          KillTree(request,response);
      }
      return true;
    }
    bool RestartTree(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res)
    {
      RCLCPP_INFO(node_->get_logger(), "TREE UID: '%u' ", req->tree_uid);
      if(uids_to_tree_info.find(req->tree_uid) != uids_to_tree_info.end())
      {
          TreeProcessInfo tree_info = uids_to_tree_info.at(req->tree_uid);
          return RosServiceRestartCall(tree_info.tree_name);
      }
      return false;
    }
    bool GetSyncBBValues (const std::shared_ptr<GetBBValuesSrv::Request> req, std::shared_ptr<GetBBValuesSrv::Response> res)
    {
      int i = 0;
      for (auto key : req->keys)
      {
        RCLCPP_INFO(node_->get_logger(), "TREE KEY %d: '%s' ", i, key.c_str());
        i++;

        //TODO 3: Get Sync BB Values
        /*if(const std::shared_ptr<BT::Blackboard::Entry> entry_ptr = sync_blackboard_ptr_->getEntry(key))
        {
            BBEntry bb_entry;
            bb_entry.key = key;
            bb_entry.type = sync_blackboard_ptr_->getEntry(key)->info.typeName();
            bb_entry.value = entry_ptr->value; 
            res->entries.push_back(bb_entry);
        }*/
      }
      return true;
    }
    bool GetTreeStatus(const std::shared_ptr<GetTreeStatusSrv::Request> req, std::shared_ptr<GetTreeStatusSrv::Response> res)
    {
      RCLCPP_INFO(node_->get_logger(), "TREE UID: '%u' ", req->tree_uid);
      return true;
    }
    bool GetAllTreeStatus(const std::shared_ptr<GetAllTreeStatusSrv::Request> req, std::shared_ptr<GetAllTreeStatusSrv::Response> res)
    {
      RCLCPP_INFO(node_->get_logger(), "GET ALL TREES STATUS");
      for (auto tree_info : uids_to_tree_info)
      {
        res->status.push_back(tree_info.second.tree_status);
      }
      return true;
    }

   
    void InitializeBlackboard(const std::string& abs_file_path, BT::Blackboard::Ptr blackboard_ptr, const bool sync_bb = false)
    {
        //TODO 4 : INIT_SYNC_BB
        /*try 
        {
            // RCLCPP_INFO(this->get_logger(), Initializing BB from YAML file %s", abs_file_path.c_str());
            YAML::Node config = YAML::LoadFile(abs_file_path);
            for(YAML::const_iterator it=config.begin();it!=config.end();++it)
            {
                const std::string& bb_key = it->first.as<std::string>();
                std::string bb_val = it->second.as<std::string>();
                
                const BT::Optional<std::string> bbentry_value_inferred_keyvalues = blackboard_ptr->replaceKeysWithStringValues(bb_val, true); // no effect if it has no key
                if(!bbentry_value_inferred_keyvalues)
                {
                    // but will complain if it has a reference to a wrong key
                    
                    RCLCPP_ERROR(this->get_logger(), "Init. of BB key %s for value %s, value inference did not succeed: %s", 
                        bb_key.c_str(), 
                        bb_val.c_str(),
                        bbentry_value_inferred_keyvalues.error().c_str());
                    continue; // and skip this init
                }
                else
                    bb_val = bbentry_value_inferred_keyvalues.value();

                RCLCPP_INFO(this->get_logger(), "Init. BB key [\"%s\"] with value \"%s\"", bb_key.c_str(), bb_val.c_str());
                // use the string here and blackboard_ptr->set(...)
                blackboard_ptr->set(bb_key, bb_val, sync_bb);
                
            }
            RCLCPP_INFO(this->get_logger(), "Initialized BB with %ld entries from YAML file %s", blackboard_ptr->getKeys().size(), abs_file_path.c_str());
        }
        catch(const YAML::Exception& ex) 
        { 
            RCLCPP_ERROR(this->get_logger(), "Initializing. BB key from file '%s' did not succeed: %s", abs_file_path.c_str(), ex.what());
        }
      */
    }

    void TreeStatusTopicCB(const TreeStatus::SharedPtr msg)   
    {
        uids_to_tree_info.at(msg->uid).tree_status = *msg;
        RCLCPP_INFO(node_->get_logger(),"New Status topic RX: Tree_name:%s New status:%s", uids_to_tree_info.at(msg->uid).tree_name.c_str() , uids_to_tree_info.at(msg->uid).tree_status.status.c_str());
    }

    bool RosServiceStopCall (std::string tree_name)
    {
        auto empty_message = std::make_shared <EmptySrv::Request>();

        rclcpp::Client<EmptySrv>::SharedPtr service_client = node_->create_client<EmptySrv>("/"+tree_name+ "/stop_tree"); 
        auto result = service_client->async_send_request(empty_message);
        if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(node_->get_logger(), "Stop Tree Service OK on %s BT", tree_name.c_str());
          return true;
        }
        RCLCPP_ERROR(node_->get_logger(),"Stop Tree Service FAILED on %s BT", tree_name.c_str());
        return false;
    }
    bool RosServiceRestartCall (std::string tree_name)
    {
        auto empty_message = std::make_shared <EmptySrv::Request>();
        rclcpp::Client<EmptySrv>::SharedPtr service_client = node_->create_client<EmptySrv>("/"+tree_name+ "/restart_tree"); 
        auto result = service_client->async_send_request(empty_message);
        if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(node_->get_logger(), "Restart Tree Service OK on %s BT", tree_name.c_str());
          return true;
        }
        RCLCPP_ERROR(node_->get_logger(),"Restart Tree Service FAILED on %s BT", tree_name.c_str());
        return false;
    }

    //Services
    rclcpp::Service<LoadTreeSrv>::SharedPtr load_tree_srv_;
    rclcpp::Service<TreeRequestSrv>::SharedPtr stop_tree_srv_;
    rclcpp::Service<TreeRequestSrv>::SharedPtr kill_tree_srv_;
    rclcpp::Service<EmptySrv>::SharedPtr kill_all_trees_srv_;
    rclcpp::Service<TreeRequestSrv>::SharedPtr restart_tree_srv_; 
    rclcpp::Service<GetBBValuesSrv>::SharedPtr get_sync_bb_values_srv_; 
    rclcpp::Service<GetTreeStatusSrv>::SharedPtr get_tree_status_srv_;
    rclcpp::Service<GetAllTreeStatusSrv>::SharedPtr get_all_trees_status_srv_;

    //Subscribers
    rclcpp::Subscription<BBEntry>::SharedPtr sync_bb_sub_;
    
    //Publishers
    rclcpp::Publisher<BBEntry>::SharedPtr sync_bb_pub_;

    //Sync BB
    BT::Blackboard::Ptr sync_blackboard_ptr_;

    //BT_FACTORY
    BT::BehaviorTreeFactory bt_factory_;

    //Save Spawned Trees information
    std::map <unsigned int, TreeProcessInfo> uids_to_tree_info;

    //Manage spawn Process using ROS LAUNCH command
    ROS2LaunchManager ros2_launch_manager;

    //Manage Trees_UIDs
    unsigned int trees_UID = 0;

    rclcpp::Node::SharedPtr node_ ;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("behavior_tree_server");

  auto bt_server = std::make_shared<BehaviorTreeServer>(nh);

  rclcpp::spin(nh);
  rclcpp::shutdown();

  return 0;
}