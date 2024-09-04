#include <ros/package.h>
#include <ros/ros.h>
#include <boost/filesystem.hpp>

#include "BehaviorTreeServer.hpp"
#include "behavior_tree_ros/3rdparty/tinyxml2/tinyxml2.h"

#include <cstdlib>
#include <signal.h>

namespace BT_ROS
{
    BehaviorTreeServer::BehaviorTreeServer () : 
        loop_rate_(private_node_handle_.param("tick_frequency", 30.0))
    {
        load_tree_srv_              = public_node_handle_.advertiseService("behavior_tree_server/load_tree", &BehaviorTreeServer::LoadTree, this);
        stop_tree_srv_              = public_node_handle_.advertiseService("behavior_tree_server/stop_tree", &BehaviorTreeServer::StopTree, this);
        restart_tree_srv_              = public_node_handle_.advertiseService("behavior_tree_server/restart_tree", &BehaviorTreeServer::RestartTree, this);
        get_tree_status_srv_        = public_node_handle_.advertiseService("behavior_tree_server/get_tree_status", &BehaviorTreeServer::StatusTree, this);
        get_all_trees_status_srv_   = public_node_handle_.advertiseService("behavior_tree_server/get_all_trees_status", &BehaviorTreeServer::StatusAllTree, this);
        
        /* SYNC_BLACKBOARD */
        sync_blackboard_ptr_ = BT::Blackboard::create();

        //Updates subscriber server side
        sync_bb_sub_ =  public_node_handle_.subscribe("/behavior_tree_server/local_update", 10, &BehaviorTreeServer::SyncBlackboardUpdateCallback, this);    
        
        //Updates republisher for all trees (put latch to true atm, because seems a good option that you receive last update from the server)
        sync_bb_pub_ = public_node_handle_.advertise<behavior_tree_ros::BBEntry>("/behavior_tree_server/broadcast_update", 10, true);  
        
    }

    BehaviorTreeServer::~BehaviorTreeServer () 
    {
        ROS_INFO("KILLING BEHAVIOR_TREE_SERVER");
    }

    void BehaviorTreeServer::SyncBlackboardUpdateCallback(const behavior_tree_ros::BBEntry& _topic_msg)
    {
        // std::cout << "BehaviorTreeServer::SyncBlackboardUpdateCallback " << 
        //     "\tkey=" << _topic_msg.key << 
        //     "\ttype=" << _topic_msg.type << 
        //     "\tvalue=" << _topic_msg.value << "\n" << std::flush;
        // bool update_successful = false;
        
        const bool void_type = (_topic_msg.type == BT::demangle(typeid(void))); // source tree does not know the type of the value
        //retrieve string converter functor
        const BT::StringConverter* string_converter_ptr = void_type? nullptr : bt_factory_.getStringConverter(_topic_msg.type);
        if(!void_type && string_converter_ptr == nullptr)
        {
            ROS_ERROR("[BTServer] Entry in Sync. BB for key [%s] has type [%s], but no string converter can be found for this type", _topic_msg.key.c_str(), _topic_msg.type.c_str());
            return;
        }

        const BT::Blackboard::Entry* entry_check_ptr = sync_blackboard_ptr_->getEntry(_topic_msg.key);
        if(entry_check_ptr == nullptr || (entry_check_ptr->port_info.missingTypeInfo() && !void_type))
        {
            // Entry not present in the BB -> First insert
            BT::Optional<BT::PortInfo> port_info_opt = bt_factory_.getPortInfo(_topic_msg.type);
            if(!port_info_opt.has_value())
            {
                ROS_ERROR("[BTServer] Entry in Sync. BB for key [%s] has type [%s], but it is an unknown type and therefore cannot be treated", _topic_msg.key.c_str(), _topic_msg.type.c_str());
                return; // type unknown
            }
            
            // Set empty entry with type info
            sync_blackboard_ptr_->setPortInfo(_topic_msg.key, port_info_opt.value());
            ROS_INFO("[BTServer] Entry in Sync. BB for key [%s] updated with type [%s]", _topic_msg.key.c_str(), BT::demangle(port_info_opt.value().type()).c_str());
        }

        //retrieve current entry in bt server bb
        const BT::Blackboard::Entry* entry_ptr = sync_blackboard_ptr_->getEntry(_topic_msg.key);
        if(entry_ptr)
        {
            //entry already present in the bb -> UPDATE
            
            if(!void_type && _topic_msg.type != BT::demangle(entry_ptr->port_info.type()))
            {
                ROS_ERROR("[BTServer] Entry in Sync. BB for key [%s] has type [%s], but receiving requests for update with type [%s]", _topic_msg.key.c_str(), BT::demangle(entry_ptr->port_info.type()).c_str(), _topic_msg.type.c_str());
                return; // type inconsistencies, don't update
            }
            
            try
            {
                if(!void_type)
                {
                    // convert from string new value
                    BT::Any new_any_value = (*string_converter_ptr)(_topic_msg.value);

                    // update it into the sync BB
                    sync_blackboard_ptr_->setAny(_topic_msg.key, std::move(new_any_value), true);
                }
                else
                    sync_blackboard_ptr_->set(_topic_msg.key, _topic_msg.value, true);
            }
            catch(const std::exception& e)
            {
                ROS_ERROR("[BTServer] Entry in Sync. Fail to update value in BB for key [%s]: %s", _topic_msg.key.c_str(), e.what());
                return;
            }

            // if(update_successful)
            {
                behavior_tree_ros::BBEntry upd_msg;
                upd_msg.key = _topic_msg.key;
                upd_msg.type = BT::demangle(entry_ptr->port_info.type());
                upd_msg.value = _topic_msg.value;
                sync_bb_pub_.publish(upd_msg);
            }
        }
    }
    
    bool BehaviorTreeServer::RosServiceStopCall (std::string tree_name)
    {
        std_srvs::Empty empty_message;
        ros::ServiceClient service_client = private_node_handle_.serviceClient<std_srvs::Empty>("/"+tree_name+ "/stop_tree");
        if (service_client.call(empty_message))
        {
            return true;
        }
        ROS_INFO("REQUEST FAILED");
        return false;

    }

    bool BehaviorTreeServer::RosServiceRestartCall (std::string tree_name)
    {
        std_srvs::Empty empty_message;
        ros::ServiceClient service_client = private_node_handle_.serviceClient<std_srvs::Empty>("/"+tree_name+ "/restart_tree");
        if (service_client.call(empty_message))
        {
            return true;
        }
        ROS_INFO("REQUEST FAILED");
        return false;

    }

    behavior_tree_ros::TreeExecutionStatus BehaviorTreeServer::RosServiceStatusCall (std::string tree_name)
    {
        StatusService::Request srv_request;
        StatusService::Response srv_response;
        ros::ServiceClient service_client = private_node_handle_.serviceClient<StatusService>("/"+tree_name+ "/status_tree");
        if (service_client.call(srv_request,srv_response))
        {
            return srv_response.status;
        }
        ROS_INFO("REQUEST FAILED");
        return srv_response.status;
    }

    bool BehaviorTreeServer::LoadTree(LoadTreeService::Request& _request, LoadTreeService::Response& _response)
    {
        //1. Extract Tree Name
        std::string tree_name;
        std::string tree_filename_tmp = _request.tree_file;
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
            ROS_ERROR("TREE FILE NOT VALID %s",tree_filename_tmp.c_str());
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

        ROS_INFO("LOADING TREE: %s",tree_name.c_str());

        trees_UID++;
        std::string param_name = "tree_name:="+tree_name;
        std::string param_file = "tree_file:="+_request.tree_file;
        std::string param_uid = "tree_uid:="+std::to_string(trees_UID);
        std::string param_debug ="tree_debug:="+std::to_string(_request.debug);

        std::string param_bb_init = "tree_bb_init:='";
        if (_request.bb_init_files.size()> 0)
        {
            param_bb_init += "[";
            for (long unsigned int i = 0; i < _request.bb_init_files.size(); i++)
            {
                param_bb_init += _request.bb_init_files[i];
                if (i != (_request.bb_init_files.size()-1))
                     param_bb_init += ",";
            }
            param_bb_init += "]'";
        }
        else
            param_bb_init += "";

        //Set default port IDs
        int server_port = 1667;
        int publisher_port = 1666;

        //Get port parameters
        if (_request.server_port > 0)
            server_port = _request.server_port;
        if (_request.publisher_port > 0)
            publisher_port = _request.publisher_port;

        std::string param_server_port = "server_port:="+std::to_string(server_port);  
        std::string param_pub_port ="publisher_port:="+std::to_string(publisher_port);
        
        ////////////////////////Version1////////////////////////
        /*std::string command = "roslaunch behavior_tree_ros behavior_tree_spawner.launch"+param_name+param_file+param_uid+param_debug+param_bb_init+param_server_port+param_pub_port;
        int result = system(command.c_str());
        if (result == -1)
        {
            ROS_ERROR("Failed to execute roslaunch command");
            return false;
        }*/
        ////////////////////////////////////////////////////////
        
        ////////////////////////Version2////////////////////////
        pid_t pid;
        try {
            pid = ros_launch_manager.start(
                 "behavior_tree_ros",
                 "behavior_tree_spawner.launch",
                 param_name.c_str(),
                 param_file.c_str(),
                 param_uid.c_str(),
                 param_debug.c_str(),
                 param_bb_init.c_str(),
                 param_server_port.c_str(),
                 param_pub_port.c_str()
                 );
        }
        catch (std::exception const &exception) {
            ROS_WARN("%s", exception.what());
            return false;
        }
        TreeProcessInfo new_process_info {tree_name,pid};
        std::string topic_name = "/"+tree_name+"/execution_status";
        new_process_info.status_subscriber =  public_node_handle_.subscribe(topic_name, 10, &BehaviorTreeServer::StatusTopicCallbackServer, this);
        uids_to_tree_info.emplace(trees_UID,new_process_info);
        ////////////////////////////////////////////////////////
        
        ROS_INFO("LOADING %s OK", tree_name.c_str());
        return true;
    }

    bool BehaviorTreeServer::StopTree(StopTreeService::Request& _request, StopTreeService::Response& _response)
    {
        if(uids_to_tree_info.find(_request.tree_uid) != uids_to_tree_info.end())
        {
            TreeProcessInfo tree_info = uids_to_tree_info.at(_request.tree_uid);
            return RosServiceStopCall(tree_info.tree_name);
        }
        return false;
    }


    bool BehaviorTreeServer::RestartTree(RestartTreeService::Request& _request, RestartTreeService::Response& _response)
    {
        if(uids_to_tree_info.find(_request.tree_uid) != uids_to_tree_info.end())
        {
            TreeProcessInfo tree_info = uids_to_tree_info.at(_request.tree_uid);
            return RosServiceRestartCall(tree_info.tree_name);
        }
        return false;
    }

    bool BehaviorTreeServer::StatusTree(StatusServiceByID::Request& _request, StatusServiceByID::Response& _response)
    {
        //Method 1. Get Status by Calling Service
        /*TreeProcessInfo tree_info = uids_to_tree_info.at(_request.tree_uid);
        _response.status = RosServiceStatusCall(tree_info.tree_name);*/

        //Method 2. Get Saved Status updated by ROS topic
        _response.status = uids_to_tree_info.at(_request.tree_uid).tree_status;

        return true;
    }
    bool BehaviorTreeServer::StatusAllTree(StatusAllService::Request& _request, StatusAllService::Response& _response)
    {
        for (auto tree_info : uids_to_tree_info)
        {
            //Method 1. Get Status by Calling Service
            //_response.status.push_back(RosServiceStatusCall(tree_info.second.tree_name));

            //2. Get Saved Status updated by ROS topic
            _response.status.push_back(tree_info.second.tree_status);
        }
        return true;
    }
    void BehaviorTreeServer::StatusTopicCallbackServer(const behavior_tree_ros::TreeExecutionStatus& _topic_msg)
    {
        uids_to_tree_info.at(_topic_msg.uid).tree_status = _topic_msg;
        ROS_INFO("New Status topic RX: Tree_name:%s New status:%s", uids_to_tree_info.at(_topic_msg.uid).tree_name.c_str() , uids_to_tree_info.at(_topic_msg.uid).tree_status.status.c_str());
    }

    /*
    bool BehaviorTreeServer::ExecuteLaunchFile (std::string tree_name, std::string tree_file)
    {
        // Call roslaunch command to run your launch file
        std::string command = "roslaunch behavior_tree_ros behavior_tree_node_execute.launch tree_name:="+tree_name+ " tree_file:="+tree_file;
        int result = system(command.c_str());
        if (result == -1)
        {
            ROS_ERROR("Failed to execute roslaunch command");
            return false;
        }
        return true;
    }*/
}
