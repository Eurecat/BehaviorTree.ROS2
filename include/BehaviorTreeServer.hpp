#ifndef BEHAVIOR_TREE_SERVER_HPP
#define BEHAVIOR_TREE_SERVER_HPP

#include <string>
#include <vector>
#include <atomic>
#include <mutex>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#include <behaviortree_cpp_v3/bt_factory.h>

#include <behavior_tree_ros/LoadTree.h>
#include <behavior_tree_ros/StopTree.h>
#include <behavior_tree_ros/BehaviorTreeAction.h>
#include <behavior_tree_ros/TreeExecutionStatus.h>
#include <behavior_tree_ros/GetTreeStatus.h>
#include <behavior_tree_ros/GetTreeStatusByID.h>
#include <behavior_tree_ros/GetAllTreesStatus.h>
#include <behavior_tree_ros/BBEntry.h>

#include "ros_launch_manager.hpp"

namespace BT_ROS
{
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
            behavior_tree_ros::TreeExecutionStatus tree_status;
            ros::Subscriber status_subscriber;
    };

    class BehaviorTreeServer final
    {
        private:

            using LoadTreeService = behavior_tree_ros::LoadTree;
            using StopTreeService  = behavior_tree_ros::StopTree;
            using StatusServiceByID  = behavior_tree_ros::GetTreeStatusByID;
            using StatusService  = behavior_tree_ros::GetTreeStatus;
            using StatusAllService  = behavior_tree_ros::GetAllTreesStatus;

        public:
            BehaviorTreeServer();
            ~BehaviorTreeServer();

        private:
            bool LoadTree(LoadTreeService::Request& _request, LoadTreeService::Response& _response);
            bool StopTree(StopTreeService::Request& _request, StopTreeService::Response& _response);
            bool StatusTree(StatusServiceByID::Request& _request, StatusServiceByID::Response& _response);
            bool StatusAllTree(StatusAllService::Request& _request, StatusAllService::Response& _response);
            bool RosServiceStopCall (std::string tree_name);
            behavior_tree_ros::TreeExecutionStatus RosServiceStatusCall (std::string tree_name);
            void StatusTopicCallbackServer(const behavior_tree_ros::TreeExecutionStatus& _topic_msg);
            void SyncBlackboardUpdateCallback(const behavior_tree_ros::BBEntry& _topic_msg);
    
        private:
            ros::NodeHandle private_node_handle_ { "~" };
            ros::NodeHandle public_node_handle_;
            ros::Rate loop_rate_;

            ros::ServiceServer get_tree_status_srv_;
            ros::ServiceServer get_all_trees_status_srv_;
            ros::ServiceServer load_tree_srv_;
            ros::ServiceServer stop_tree_srv_;
            BT::Blackboard::Ptr sync_blackboard_ptr_ ;
            //Manage spawn Process using ROS LAUNCH command
            ROSLaunchManager ros_launch_manager;
            //Save Spawned Trees information
            std::map <unsigned int, TreeProcessInfo> uids_to_tree_info;
            //Manage Trees_UIDs
            unsigned int trees_UID = 0;

            BT::BehaviorTreeFactory bt_factory_;
            
            //BB Sync Publishers/Subscribers
            ros::Publisher sync_bb_pub_;
            ros::Subscriber sync_bb_sub_;
    };



} 



#endif //BEHAVIOR_TREE_SERVER_HPP