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
        get_tree_status_srv_        = public_node_handle_.advertiseService("behavior_tree_server/get_tree_status", &BehaviorTreeServer::StatusTree, this);
        get_all_trees_status_srv_   = public_node_handle_.advertiseService("behavior_tree_server/get_all_trees_status", &BehaviorTreeServer::StatusAllTree, this);
        ROS_INFO("BEHAVIOR TREE SERVER ON");
    }
    BehaviorTreeServer::~BehaviorTreeServer () 
    {
        ROS_INFO("KILLING BEHAVIOR_TREE_SERVER");
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
        std::string param_name = " tree_name:="+tree_name;
        std::string param_file = " tree_file:="+_request.tree_file;
        std::string param_uid = " tree_uid:="+std::to_string(trees_UID);
        std::string param_debug =" tree_debug:="+std::to_string(_request.debug);
        std::string param_bb_init = " tree_bb_init:="+_request.bb_init_file;
        std::string param_server_port = " server_port:="+std::to_string(_request.server_port);  
        std::string param_pub_port =" publisher_port:="+std::to_string(_request.publisher_port);
        
        ////////////////////////Version1////////////////////////
       /* std::string command = "roslaunch behavior_tree_ros behavior_tree_spawner.launch"+param_name+param_file+param_uid+param_debug+param_bb_init+param_server_port+param_pub_port;
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
                 "behavior_tree_ros", "behavior_tree_spawner.launch",
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
        TreeProcessInfo tree_info = uids_to_tree_info.at(_request.tree_uid);
        return RosServiceStopCall(tree_info.tree_name);
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
