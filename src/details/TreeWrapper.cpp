#include "behavior_tree_ros/details/TreeWrapper.hpp"

#include "yaml-cpp/yaml.h"
namespace BT_ROS
{
    void TreeWrapper::InitializeStatusPublisher(ros::NodeHandle& _public_node_handle, std::string tree_name)
    {
        std::string transition_topic_name = identifier_ == "service" ? "/"+tree_name+"/transition_status" : "/"+tree_name+"/transition_status_" + identifier_;
        std::string status_topic_name = identifier_ == "service" ? "/"+tree_name+"/execution_status" : "/"+tree_name+"/execution_status_" + identifier_;
        bt_transition_publisher_ = _public_node_handle.advertise<behavior_tree_ros::Transition>(transition_topic_name, 1);
        bt_execution_status_publisher_ = _public_node_handle.advertise<behavior_tree_ros::TreeExecutionStatus>(status_topic_name, 100, true);
        tree_name_ = tree_name;
    }

    void TreeWrapper::BuildTree(const std::string& _tree_file, BT::BehaviorTreeFactory& _bt_factory, 
        const bool debug, const std::vector<std::string>& bb_init_abs_filepaths)
    {
        ResetLoggers();
        
        BT::Blackboard::Ptr blackboard_ptr = BT::Blackboard::create();

        if(bb_init_abs_filepaths.size() > 0)
        {
            for(const auto& bb_init_abs_filepath: bb_init_abs_filepaths)
            {
                if(bb_init_abs_filepath.length() < 3) continue;

                try 
                {
                    ROS_INFO("Initializing BB from YAML file %s", bb_init_abs_filepath.c_str());
                    YAML::Node config = YAML::LoadFile(bb_init_abs_filepath);
                    for(YAML::const_iterator it=config.begin();it!=config.end();++it)
                    {
                        const std::string& bb_key = it->first.as<std::string>();
                        std::string bb_val = it->second.as<std::string>();
                        const BT::Optional<std::string> bbentry_value_inferred_keyvalues = blackboard_ptr->replaceKeysWithStringValues(bb_val, true); // no effect if it has no key
                        if(!bbentry_value_inferred_keyvalues)
                        {
                            // but will complain if it has a reference to a wrong key
                            ROS_ERROR("Init. of BB key %s for value %s did not succeed: %s", 
                                bb_key.c_str(), 
                                bb_val.c_str(),
                                bbentry_value_inferred_keyvalues.error().c_str());
                            continue; // and skip this init
                        }
                        else
                            bb_val = bbentry_value_inferred_keyvalues.value();

                        ROS_INFO("Init. BB key [\"%s\"] with value \"%s\"", bb_key.c_str(), bb_val.c_str());
                        // use the string here and blackboard_ptr->set(...)
                        blackboard_ptr->set(bb_key, bb_val);
                    }
                }
                catch(const YAML::Exception& ex) 
                { 
                    ROS_ERROR("Init. BB key from file '%s' did not succeed: %s", bb_init_abs_filepath.c_str(), ex.what());
                }
            }
        }

        // Wait between creating and executing the Tree to fully initialize ROS publishers
        auto temp_tree = std::make_unique<BT::Tree>(_bt_factory.createTreeFromFile(_tree_file, blackboard_ptr));
        ros::Duration(0.5).sleep();
        tree_.swap(temp_tree);
        ROS_INFO("Init. Tree. Blackboard debug message:");
        tree_->rootBlackboard()->debugMessage();
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
        {
            bt_logger_transition_rostopic_ = std::make_unique<BT_ROS::RosTopicTransitionLogger>(*tree_, bt_transition_publisher_);
            bt_logger_status_rostopic_  = std::make_unique<BT_ROS::RosTopicStatusLogger>(*tree_, bt_execution_status_publisher_,tree_uid_,tree_name_,tree_filename_,execution_time_);
        }

        #ifdef BEHAVIOR_TREE_CPP_ZMQ
        // Set default port for tree called with service and use a different port for the action one
        // TODO: Even if publisher ports are different, only one instance of ZMQ is allowed, check engine
       // unsigned publisher_port = identifier_ == "service" ? 1666 : 1665;
       // unsigned server_port = identifier_ == "service" ? 1667 : 1668;
        if(_enable_zmq)
            try
            {
            
                bt_logger_zmq_ = std::make_unique<BT::PublisherZMQ>(*tree_, 25, publisher_port_,server_port_);
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
        bt_logger_transition_rostopic_.reset();
        bt_logger_status_rostopic_.reset();
        #ifdef BEHAVIOR_TREE_CPP_ZMQ
        bt_logger_zmq_.reset();
        #endif
    }

    void TreeWrapper::PublishExecutionStatus(bool error, std::string error_data)
    {
        behavior_tree_ros::TreeExecutionStatus status_msg {};
        status_msg.time_start = execution_time_;
        status_msg.time = ros::Time::now();
        status_msg.uid = tree_uid_;
        status_msg.name = tree_name_;
        status_msg.file = tree_filename_;
        if (!error)
        {
            switch(status_)
            {
                case BT::NodeStatus::FAILURE:
                    status_msg.status  = "FINISHED";
                    status_msg.data = "FAILURE";
                    break;
                case BT::NodeStatus::RUNNING:
                    status_msg.status  = "RUNNING";
                    break;
                case BT::NodeStatus::SUCCESS:
                    status_msg.status  = "FINISHED";
                    status_msg.data = "SUCCESS";
                    break;
                case BT::NodeStatus::PAUSED:
                    status_msg.status  = "PAUSED";
                    break;
                default:
                    status_msg.status  = "IDLE";
                    break;
            }
        }
        else
        {
            status_msg.status    = "CRASHED";
            status_msg.data      = error_data;
        }
        bt_execution_status_publisher_.publish(status_msg);
    }

    void TreeWrapper::SyncBlackboardUpdateCallback(const behavior_tree_ros::BBEntry& _topic_msg, const BT::BehaviorTreeFactory* bt_factory_ptr)
    {
        if(!bt_factory_ptr) return;

        std::cout << "[BTWrapper "<<tree_name_<<"]::SyncBlackboardUpdateCallback " << 
            "\tkey=" << _topic_msg.key << 
            "\ttype=" << _topic_msg.type << 
            "\tvalue=" << _topic_msg.value << "\n" << std::flush;
        // bool update_successful = false;
        
        const BT::StringConverter* from_string_converter_ptr = bt_factory_ptr->getStringConverter(_topic_msg.type);
        
        //check string converter functor
        if(from_string_converter_ptr == nullptr)
        {
            ROS_ERROR("[BTWrapper %s] Entry in Sync. BB for key [%s] has type [%s], but no string converter can be found for this type", 
                tree_name_.c_str(), _topic_msg.key.c_str(), _topic_msg.type.c_str());
            return;
        }

        //retrieve current entry in bt server bb
        const BT::Blackboard::Entry* entry_ptr = tree_->rootBlackboard()->getEntry(_topic_msg.key);

        if(entry_ptr && entry_ptr->isSync())
        {
            // if(entry_ptr->port_info.missingTypeInfo()) is it necessary???
            // {
            //     BT::Optional<BT::PortInfo> port_info_opt = bt_factory_ptr->getPortInfo(_topic_msg.type);
            //     if(!port_info_opt.has_value())
            //     {
            //         ROS_ERROR("[BTWrapper %s] Entry in Sync. BB for key [%s] has type [%s], but it is an unknown type and therefore cannot be treated", tree_name_.c_str(), _topic_msg.key.c_str(), _topic_msg.type.c_str());
            //         return; // type unknown
            //     }
            //     tree_->rootBlackboard()->setPortInfo(_topic_msg.key, port_info_opt.value());
            // }            

            if(!entry_ptr->port_info.missingTypeInfo() && _topic_msg.type != BT::demangle(entry_ptr->port_info.type())) //TODO evaluate strictness and checks to be made here
            {
                ROS_ERROR("[BTWrapper %s]. Entry in Sync. BB for key [%s] has type [%s], but receiving requests for update with type [%s]",
                    tree_name_.c_str(), _topic_msg.key.c_str(), BT::demangle(entry_ptr->port_info.type()).c_str(), _topic_msg.type.c_str());
                return; // type inconsistencies, don't update
            }
            
            // convert from string new value
            BT::Any new_any_value = (*from_string_converter_ptr)(_topic_msg.value);
            
            std::cout << "[BTWrapper "<<tree_name_<<"]::SyncBlackboardUpdateCallback built new_any_value with type " << BT::demangle(new_any_value.type()) << " \n" << std::flush;
            // update it into the sync BB
            tree_->rootBlackboard()->setAny(_topic_msg.key, std::move(new_any_value), true);

            std::cout << "[BTWrapper "<<tree_name_<<"]::SyncBlackboardUpdateCallback updated value in BB for key [" << _topic_msg.key << "] \n" << std::flush;
            // update_successful = true;
        }
    }
}