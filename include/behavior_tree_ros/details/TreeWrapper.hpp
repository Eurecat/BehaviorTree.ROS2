#ifndef TREE_WRAPPER_ROS_HPP
#define TREE_WRAPPER_ROS_HPP

#include <string>
#include <vector>
#include <chrono>
#include <sstream>
#include <ctime>
#include <atomic>

#include <ros/ros.h>
#include "behavior_tree_ros/Transition.h"
#include "behavior_tree_ros/TreeExecutionStatus.h"
#include <behavior_tree_ros/BBEntry.h>

#include <behaviortree_cpp_v3/bt_factory.h>

#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>
#ifdef BEHAVIOR_TREE_CPP_ZMQ
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#endif

#include "bt_rostopic_logger.h"
#include <zmq.hpp>
#include <behaviortree_cpp_v3/flatbuffers/bt_flatbuffer_helper.h>
namespace BT_ROS
{
    class TreeWrapper final
    {
        public:
            TreeWrapper(const std::string& _identifier) : context_(1), client_sub_(context_, ZMQ_SUB), client_pub_(context_, ZMQ_PUB) , identifier_(_identifier) {}  ;
            ~TreeWrapper() = default;

            void InitializeStatusPublisher(ros::NodeHandle& _public_node_handle, std::string tree_name);

            void BuildTree(const std::string& _xml_file, BT::BehaviorTreeFactory& _bt_factory, 
                const bool debug = false, const std::vector<std::string>& bb_init_abs_filepaths = {});
            void RemoveTree();

            bool ResetTree()
            {
                if(IsTreeLoaded() && AreLoggersInitialized())
                {
                    tree_->haltTree();
                    return true;
                }
                else
                    return false;
            }

            bool HasExecutionTerminated(){return executed_;}
            void SetExecuted(bool executed){executed_ = executed;};


            void InitializeLoggers(const bool& _enable_cout, const bool& _enable_minitrace, const bool& _enable_file,
                                   const bool& _enable_topic, const bool& _enable_zmq, const std::string& _log_folder);
            void ResetLoggers();
            void PublishExecutionStatus(bool error=false, std::string error_data="");
            bool IsTreeLoaded() { return !!tree_; };
            bool AreLoggersInitialized() { return loggers_initialized_.load(); };
            size_t TreeNodesCount() { return tree_->nodes.size();}
            BT::NodeStatus tickTree() { return tree_->tickRoot(); };
            BT::Blackboard::SerializedEntriesMap getKeysValueToSync(){return tree_->rootBlackboard()->getKeysValueToSync(true);};
            void SyncBlackboardUpdateCallback(const behavior_tree_ros::BBEntry& _topic_msg, const BT::BehaviorTreeFactory* = nullptr);
            std::string execution_tree_status_ {};
            std::string execution_tree_error_ {};
            std::string tree_filename_ {};
            std::string tree_name_ {};
            std::vector<std::string> tree_bb_init_ {};
            bool tree_debug_ {false};
            bool tree_auto_restart_ {false};
            ros::Time execution_time_;
            unsigned int tree_uid_;
            unsigned server_port_;
            unsigned publisher_port_;
            BT::NodeStatus status_ { BT::NodeStatus::IDLE };

            zmq::context_t context_;
            zmq::socket_t client_sub_;
            zmq::socket_t client_pub_;
            std::thread thread_rx;
            std::thread thread_tx;
        private:
            std::unique_ptr<BT::Tree> tree_;
            bool executed_{false};

            std::unique_ptr<BT::StdCoutLogger>   bt_logger_cout_;
            std::unique_ptr<BT::FileLogger>      bt_logger_file_;
            std::unique_ptr<BT::MinitraceLogger> bt_logger_trace_;
            #ifdef BEHAVIOR_TREE_CPP_ZMQ
            std::unique_ptr<BT::PublisherZMQ>    bt_logger_zmq_;
            #endif

            //Transition Publisher
            ros::Publisher bt_transition_publisher_;
            std::unique_ptr<BT_ROS::RosTopicTransitionLogger> bt_logger_transition_rostopic_;

            //Status Publisher
            ros::Publisher bt_execution_status_publisher_;
            std::unique_ptr<BT_ROS::RosTopicStatusLogger> bt_logger_status_rostopic_;

            // Var to differenciate between service and action tree
            std::string identifier_;

            std::atomic_bool loggers_initialized_{false};
    };
}
#endif