#ifndef TREE_WRAPPER_ROS_HPP
#define TREE_WRAPPER_ROS_HPP

#include <string>
#include <chrono>
#include <sstream>
#include <ctime>

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
    class TreeWrapper final
    {
        public:
            TreeWrapper(const std::string& _identifier) : identifier_(_identifier) {};
            ~TreeWrapper() = default;

            void InitializeStatusPublisher(ros::NodeHandle& _public_node_handle);

            void BuildTree(const std::string& _xml_file, BT::BehaviorTreeFactory& _bt_factory);
            void RemoveTree();

            void InitializeLoggers(const bool& _enable_cout, const bool& _enable_minitrace, const bool& _enable_file,
                                   const bool& _enable_topic, const bool& _enable_zmq, const std::string& _log_folder);
            void ResetLoggers();

            bool IsTreeLoaded() { return !!tree_; };
            BT::NodeStatus tickTree() { return tree_->tickRoot(); };

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
    };
}
#endif