#ifndef ROS_TREE_HPP
#define ROS_TREE_HPP

#include <string>
#include <memory>

#include <ros/ros.h>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>

#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_file_logger.h>
#include <behaviortree_cpp/loggers/bt_minitrace_logger.h>
#include <behaviortree_cpp/loggers/bt_zmq_publisher.h>

namespace UPO
{
    class ROSTree final
    {
        public:
            using Status = BT::NodeStatus;

        public:
            ROSTree(const BT::BehaviorTreeFactory& _factory, const std::string& _xml_tree_file, ros::NodeHandle& _node_handle);

            ROSTree(const BT::BehaviorTreeFactory& _factory, const std::string& _xml_tree_file, bool _enable_cout,
                    bool _enable_zmq, bool _enable_file, bool _enable_minitrace, const std::string& _log_folder);

            ~ROSTree() = default;

            Status Tick();

        private:
            void InitializeLoggers(bool _enable_cout, bool _enable_zmq, bool _enable_file,
                                   bool _enable_minitrace, const std::string& _log_folder);

        private:
            BT::Tree bt_tree_;

            std::unique_ptr<BT::StdCoutLogger>   bt_logger_cout_;
            std::unique_ptr<BT::FileLogger>      bt_logger_file_;
            std::unique_ptr<BT::MinitraceLogger> bt_logger_trace_;
            std::unique_ptr<BT::PublisherZMQ>    bt_logger_zmq_;
    };
}

#endif
