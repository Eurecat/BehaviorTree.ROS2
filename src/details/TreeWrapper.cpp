#include "behavior_tree_ros/details/TreeWrapper.hpp"

#include "yaml-cpp/yaml.h"
namespace BT_ROS
{
    void TreeWrapper::InitializeStatusPublisher(ros::NodeHandle& _public_node_handle)
    {
        std::string topic_name = identifier_ == "service" ? "bt_status" : "bt_" + identifier_ + "_status";
        bt_status_publisher_ = _public_node_handle.advertise<std_msgs::String>(topic_name, 1);
    }

    void TreeWrapper::BuildTree(const std::string& _tree_file, BT::BehaviorTreeFactory& _bt_factory, 
        const bool debug, const std::string& bb_init_abs_filepath)
    {
        ResetLoggers();
        
        BT::Blackboard::Ptr blackboard_ptr = BT::Blackboard::create();
        
        if(bb_init_abs_filepath.length() > 0)
        {
             try {
                ROS_INFO("Initializing BB from YAML file %s", bb_init_abs_filepath.c_str());
                YAML::Node config = YAML::LoadFile(bb_init_abs_filepath);
                for(YAML::const_iterator it=config.begin();it!=config.end();++it)
                {
                    const std::string& bbentry_key = it->first.as<std::string>();
                    std::string bbentry_value = it->second.as<std::string>();
                    if(BT::TreeNode::isBlackboardPointer(bbentry_value))
                    {
                        const auto opt_val = blackboard_ptr->getAsString(static_cast<std::string>(BT::TreeNode::stripBlackboardPointer(bbentry_value)).c_str());
                        if(!opt_val)
                        {
                            ROS_ERROR("Init. BB key %s did not succeed because it references entry %s which is unknown yet", bbentry_key.c_str(), static_cast<std::string>(BT::TreeNode::stripBlackboardPointer(bbentry_value)).c_str());
                            continue;
                        }
                        bbentry_value = opt_val.value();
                    }
                    ROS_INFO("Init. BB key [\"%s\"] with value \"%s\"", bbentry_key.c_str(), bbentry_value.c_str());
                    // use the string here and blackboard_ptr->set(...)
                    blackboard_ptr->set(bbentry_key, bbentry_value);
                }
            }
            catch(const YAML::Exception& ex) 
            { 
                ROS_ERROR("Init. BB key: %s", ex.what());
                /* */ 
            }
        }

        // Wait between creating and executing the Tree to fully initialize ROS publishers
        auto temp_tree = std::make_unique<BT::Tree>(_bt_factory.createTreeFromFile(_tree_file, blackboard_ptr));
        ros::Duration(0.5).sleep();
        tree_.swap(temp_tree);
        if(debug) tree_->setDebug(); // set tree in debug mode
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
            bt_logger_file_ = std::make_unique<BT::FileLogger>(*tree_, log_file.c_str(), 20, true);
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
        loggers_initialized_ = true;
    }

    void TreeWrapper::ResetLoggers()
    {
        loggers_initialized_ = false;
        bt_logger_cout_.reset();
        bt_logger_trace_.reset();
        bt_logger_file_.reset();
        bt_logger_rostopic_.reset();
        #ifdef BEHAVIOR_TREE_CPP_ZMQ
        bt_logger_zmq_.reset();
        #endif
    }
}