#ifndef TREE_WRAPPER_ROS_HPP
#define TREE_WRAPPER_ROS_HPP

#include <string>
#include <chrono>
#include <sstream>
#include <ctime>
#include <atomic>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <behaviortree_cpp_v3/bt_factory.h>

#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>

#ifdef BEHAVIOR_TREE_CPP_ZMQ
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#endif

#include "bt_rostopic_logger.h"

namespace BT_ROS
{
    static int tree_UID = 1;
    class TreeWrapper final
    {
        public:
            TreeWrapper(const std::string& _identifier) : identifier_(_identifier) {
                tree_UID_ = tree_UID;
                tree_UID++;
            };
            ~TreeWrapper() = default;

            void InitializeStatusPublisher(ros::NodeHandle& _public_node_handle, uint8_t uid = 0);

            void BuildTree(const std::string& _xml_file, BT::BehaviorTreeFactory& _bt_factory, 
                const bool debug = false, const std::string& bb_init_abs_filepath = "");
            void RemoveTree();

            void InitializeLoggers(const bool& _enable_cout, const bool& _enable_minitrace, const bool& _enable_file,
                                   const bool& _enable_topic, const bool& _enable_zmq, const std::string& _log_folder);
            void ResetLoggers();

            bool IsTreeLoaded() { return !!tree_; };
            bool AreLoggersInitialized() { return loggers_initialized_.load(); };
            BT::NodeStatus tickTree() { return tree_->tickRoot(); };
            uint8_t tree_UID_;
            std::string execution_tree_status {};
            std::string execution_tree_error {};
            std::string tree_filename {};
            ros::Time execution_time;
            unsigned server_port_;
            unsigned publisher_port_;
        private:
            std::unique_ptr<BT::Tree> tree_;

            std::unique_ptr<BT::StdCoutLogger>   bt_logger_cout_;
            std::unique_ptr<BT::FileLogger>      bt_logger_file_;
            std::unique_ptr<BT::MinitraceLogger> bt_logger_trace_;
            #ifdef BEHAVIOR_TREE_CPP_ZMQ
            std::unique_ptr<BT::PublisherZMQ>    bt_logger_zmq_;
            #endif

            ros::Publisher bt_status_publisher_;
            std::unique_ptr<BT_ROS::RosTopicLogger> bt_logger_rostopic_;

            // Var to differenciate between service and action tree
            std::string identifier_;

            std::atomic_bool loggers_initialized_{false};
    };
}
#endif