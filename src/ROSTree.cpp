#include <chrono>
#include <sstream>
#include <iomanip>
#include <ctime>

#include <Blackboard/blackboard_local.h>

#include "ROSTree.hpp"

namespace UPO
{
    ROSTree::ROSTree(const BT::BehaviorTreeFactory& _factory, const std::string& _xml_tree_file, bool _enable_cout,
                     bool _enable_zmq, bool _enable_file, bool _enable_minitrace, const std::string& _log_folder)
    {
        bt_tree_ = BT::buildTreeFromFile(_factory, _xml_tree_file, BT::Blackboard::create<BT::BlackboardLocal>());
        InitializeLoggers(_enable_cout, _enable_zmq, _enable_file, _enable_minitrace, _log_folder);
    }

    ROSTree::ROSTree(const BT::BehaviorTreeFactory& _factory, const std::string& _xml_tree_file, ros::NodeHandle& _node_handle) :
        ROSTree(_factory, _xml_tree_file, _node_handle.param("enable_cout_log", false), _node_handle.param("enable_zmq_pub", false),
                _node_handle.param("enable_file_log", false), _node_handle.param("enable_minitrace_log", false), _node_handle.param<std::string>("log_folder", {}))
    {}

    ROSTree::Status ROSTree::Tick()
    {
        if(!bt_tree_.root_node) { throw std::runtime_error { "Calling tick on null tree" }; }
        return bt_tree_.root_node->executeTick();
    }

    void ROSTree::InitializeLoggers(bool _enable_cout, bool _enable_zmq, bool _enable_file, bool _enable_minitrace, const std::string& _log_folder)
    {
        std::stringstream file_base;
        const auto& current_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

        file_base << _log_folder << "behavior_tree_ros-" << std::put_time(std::localtime(&current_time), "%F-%R");
        const auto& log_file           = file_base.str() + ".fbl";
        const auto& log_minitrace_file = file_base.str() + ".json";

        if(_enable_cout)      { bt_logger_cout_  = std::make_unique<BT::StdCoutLogger>(bt_tree_.root_node); }
        if(_enable_zmq)       { bt_logger_zmq_   = std::make_unique<BT::PublisherZMQ>(bt_tree_.root_node);  }
        if(_enable_file)      { bt_logger_file_  = std::make_unique<BT::FileLogger>(bt_tree_.root_node, log_file.c_str());  }
        if(_enable_minitrace) { bt_logger_trace_ = std::make_unique<BT::MinitraceLogger>(bt_tree_.root_node, log_minitrace_file.c_str());  }
    }
}
