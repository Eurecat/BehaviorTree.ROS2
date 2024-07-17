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

        //TEST 
        blackboard_ptr->setSyncKey("sync_val","0");

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
                InitSyncBB();
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
        //if(client_pub_.connected()) client_pub_.disconnect("tcp://127.0.0.1:2000");
        //if(client_sub_.connected()) client_sub_.disconnect("tcp://127.0.0.1:2001");
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

    void TreeWrapper::CheckSyncPortsChanged ()
    {
        std::unordered_map<std::string, std::string> sync_ports_changed = tree_->rootBlackboard()->get_sync_values_changed();
        //SEND SYNC DATA
        if (!sync_ports_changed.empty())
        {
            std::cout << "CheckSyncPortsChanged OK " << std::endl;
            TransmitNewBBDataChanged(sync_ports_changed);
        }
    }
    void TreeWrapper::TransmitNewBBDataChanged(std::unordered_map<std::string, std::string> sync_ports_changed)
    {
         std::cout << "TransmitNewBBDataChanged START " << std::endl;

        for (auto syncport : sync_ports_changed)
            std::cout << "TransmitNewBBDataChanged 1 KEY: " << syncport.first << " VAL: " << syncport.second << std::endl;

        //CONSTRUCT SERIALIZED MESSAGE
        std::string message_str = sync_ports_changed.size() > 0? BT::flattenValueMap(sync_ports_changed) : "";

        std::cout << "TransmitNewBBDataChanged 2 STR: " << message_str << std::endl;
        char* msg  = new char[message_str.size()];
        msg = message_str.data();
        zmq::message_t message(msg, strlen(msg));
        std::cout << "TransmitNewBBDataChanged 3 SIZE " << strlen(msg) << std::endl;
        //SEND MESSAGE
        client_pub_.send(message, zmq::send_flags::none);
        std::cout << "TransmitNewBBDataChanged OK" << std::endl;
    }
    void TreeWrapper::InitSyncBB()  
    {
        std::cout << "STARTING InitSyncBB ... " << std::endl;
        int timeout_ms = 100;
        const std::string endpoint = "tcp://127.0.0.1:2001";
        client_sub_.setsockopt(ZMQ_RCVTIMEO,&timeout_ms, sizeof(int) );
        client_sub_.connect(endpoint);

        std::cout << "client_sub_  CONNECTED" <<  client_sub_.connected() << std::endl;
        std::cout << "InitSyncBB 1 " << std::endl;
        thread_rx = std::thread([this]()
        {
            zmq::message_t req;
            bool active_client = true;
            while (active_client)
            {
                try
                {
                   // std::cout << "ReceiveNewBBDataChanged 2 " << std::endl;
                    zmq::recv_result_t received = client_sub_.recv(req);
                    if (received)
                    {
                        //RECEIVE BB_UPDATES from Server
                        //size_t received_data_size = received.value();
                        const char* req_data_raw = static_cast<const char*>(req.data());
                        std::cout << "ReceiveNewBBDataChanged 1 " << std::endl;
                        //DESERIALIZE DATA
                        std::unordered_map<std::string, std::string> PortsValueMap = unflattenValueMap(req_data_raw);
                        std::cout << "ReceiveNewBBDataChanged 2 " << std::endl;
                        for (auto portvalue : PortsValueMap)
                        {
                             std::cout << "ReceiveNewBBDataChanged 3 " << std::endl;
                            UpdateBlackBoardPortFromServer(portvalue.first,portvalue.second);
                        }
                         std::cout << "ReceiveNewBBDataChanged 4 " << std::endl;
                    }
                }
                catch (zmq::error_t& err)
                {
                    if (err.num() == ETERM)
                    {
                        std::cout << "[ZMQ CLIENT] Client quitting." << std::endl;
                    }
                    std::cout << "[ZMQ CLIENT]  just died. Exception " << err.what() << std::endl;
                    active_client = false;
                }
                req.rebuild();//clean req message after processing
            }
        });
         std::cout << "InitSyncBB 2 " << std::endl;
        thread_tx = std::thread([this]()
        {
            //SEND SYNC DATA Changed to BT server node
            //CONNECT TO SERVER
            int timeout_ms = 100;
             const std::string endpoint = "tcp://127.0.0.1:2000";
            client_pub_.setsockopt(ZMQ_RCVTIMEO,&timeout_ms, sizeof(int) );
            client_pub_.connect(endpoint);

            std::cout << "client_pub_  CONNECTED" <<  client_pub_.connected() << std::endl;

            bool active_client = true;
            while (active_client)
            {
                CheckSyncPortsChanged();
                usleep(100);
            }
        });
         std::cout << "InitSyncBB 3" << std::endl;
    }
    void TreeWrapper::UpdateBlackBoardPortFromServer(std::string key, std::string val)
    {
        if (tree_->rootBlackboard()->getEntry(key) != nullptr /*&& tree_->rootBlackboard()->getEntry(key)->getSync()*/)
            tree_->rootBlackboard()->set(key,val,true);
    }
    std::unordered_map<std::string, std::string> TreeWrapper::unflattenValueMap(const char* req_data_raw)
    {
        std::unordered_map<std::string, std::string> sync_ports_changed;
        std::string req_data_raw_str(req_data_raw);
        auto parts = BT::splitString(req_data_raw_str, ',');
        for (auto part : parts)
        {
            //Extract KEY VALUE
            std::cout << "RX " << part << std::endl;
            auto key = part.substr(part.find_first_of("{")+1, part.find_first_of(":")-part.find_first_of("{")-1);
            auto val = part.substr(1+part.find_first_of(":"), part.find_last_of("}")-part.find_first_of(":")-1);
            std::cout << "key" << key << std::endl;
            std::cout << "val" << val << std::endl;
            //Replace
            sync_ports_changed.insert(std::make_pair(std::string{key},std::string{key}));

        }
        for(auto& it : sync_ports_changed)
        {
            // remove escape characters in values
            it.second = boost::replace_all_copy(it.second,  "\\{", "{");
            it.second = boost::replace_all_copy(it.second,  "\\}", "}");
        }
        return sync_ports_changed;
    }

}