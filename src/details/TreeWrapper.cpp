#include "behavior_tree_ros/details/TreeWrapper.hpp"

namespace BT_ROS
{
    void TreeWrapper::InitializeStatusPublisher(ros::NodeHandle& _public_node_handle)
    {
        std::string topic_name = identifier_ == "service" ? "bt_status" : "bt_" + identifier_ + "_status";
        bt_status_publisher_ = _public_node_handle.advertise<std_msgs::String>(topic_name, 1);
    }

    void TreeWrapper::BuildTree(const std::string& _tree_file, BT::BehaviorTreeFactory& _bt_factory)
    {
        // Wait between creating and executing the Tree to fully initialize ROS publishers
        auto temp_tree = std::make_unique<BT::Tree>(_bt_factory.createTreeFromFile(_tree_file));
        ros::Duration(0.5).sleep();
        tree_.swap(temp_tree);
    }

    void TreeWrapper::RemoveTree()
    {
        ResetLoggers();
        tree_.reset();
    }

    void TreeWrapper::InitializeLoggers(const bool& _enable_cout, const bool& _enable_minitrace, const bool& _enable_file,
                                    const bool& _enable_topic, const bool& _enable_zmq, const std::string& _log_folder)
    {
        if(!tree_ || !tree_->rootNode()) { return; }

        // behavior_tree_core complains if two instances of the same logger exist at the same time,
        // so the pointer is resetted explictly first
        ResetLoggers();

        std::stringstream file_base;
        const auto& current_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

        std::string log_folder = _log_folder.back() == '/' ? _log_folder : _log_folder + "/";
        file_base << log_folder << "behavior_tree_ros-" << identifier_ << "_tree-" << std::put_time(std::localtime(&current_time), "%F-%R");;
        const auto& log_file       = file_base.str() + ".fbl";
        const auto& minitrace_file = file_base.str() + ".json";

        if(_enable_cout)
            try
            {
                bt_logger_cout_ = std::make_unique<BT::StdCoutLogger>(*tree_);
            }
            catch(const BT::LogicError& ex)
            {
                ROS_WARN("Error initializing Cout logger for %s: %s", identifier_.c_str(), ex.what());
            }
        if(_enable_minitrace)
            try
            {
                bt_logger_trace_ = std::make_unique<BT::MinitraceLogger>(*tree_, minitrace_file.c_str());
            }
            catch(const BT::LogicError& ex)
            {
                ROS_WARN("Error initializing Minitrace logger for %s: %s", identifier_.c_str(), ex.what());
            }
        if(_enable_file)
            bt_logger_file_ = std::make_unique<BT::FileLogger>(*tree_, log_file.c_str());
        if(_enable_topic)
            bt_logger_rostopic_ = std::make_unique<BT_ROS::RosTopicLogger>(*tree_, bt_status_publisher_);

        #ifdef BEHAVIOR_TREE_CPP_ZMQ
        // Set default port for tree called with service and use a different port for the action one
        // TODO: Even if publisher ports are different, only one instance of ZMQ is allowed, check engine
        unsigned publisher_port = identifier_ == "service" ? 1666 : 1665;

        if(_enable_zmq)
            try
            {
                bt_logger_zmq_ = std::make_unique<BT::PublisherZMQ>(*tree_, 25, publisher_port);
            }
            catch(const BT::LogicError& ex)
            {
                ROS_WARN("Error initializing ZMQ logger for %s: %s", identifier_.c_str(), ex.what());
            }
        #else
        ROS_WARN("ZMQ logging is enabled but behavior_tree_core was not compiled with ZMQ support.");
        #endif
    }

    void TreeWrapper::ResetLoggers()
    {
        bt_logger_cout_.reset();
        bt_logger_trace_.reset();
        bt_logger_file_.reset();
        bt_logger_rostopic_.reset();
        #ifdef BEHAVIOR_TREE_CPP_ZMQ
        bt_logger_zmq_.reset();
        #endif
    }
}