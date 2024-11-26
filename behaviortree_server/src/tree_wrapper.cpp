#include "tree_wrapper.hpp"

#include "yaml-cpp/yaml.h"

namespace BT
{

  TreeWrapper::TreeWrapper(const rclcpp::Node::SharedPtr& node)
    : node_(node)
  {
    global_blackboard_ = BT::Blackboard::create();
  }

  TreeWrapper::~TreeWrapper() {}

  bool TreeWrapper::ResetTree()
  {
      if(IsTreeLoaded() )
      {
          tree_.haltTree();
          is_tree_loaded_ = false;
          return true;
      }
      return false;
  }

  void TreeWrapper::RemoveTree()
  {
     ResetTree();
  }

  void TreeWrapper::InitGrootV2Publisher()
  {
    //TODO: NEW GROOT
    groot_publisher_.reset();
    groot_publisher_ = std::make_shared<BT::Groot2Publisher>(tree_, tree_server_port_);
  }

  size_t TreeWrapper::TreeNodesCount() 
  {
      std::vector<const TreeNode*> nodes;
      size_t nodes_count = 0;
      for(auto const& subtree : tree_.subtrees)
      {
        for(auto const& node : subtree->nodes)
        {
          nodes_count += subtree->nodes.size();
        }
      }
      return nodes_count;
  }

  TreeStatus TreeWrapper::buildTreeExecutionStatus()
  {
      TreeStatus tree_status_msg;

      static const auto to_msg_status = [](const BT::NodeStatus& bt_status)
      {
          auto status { TreeStatus::IDLE };

          switch(bt_status)
          {
              case BT::NodeStatus::FAILURE:
                  status = TreeStatus::FAILURE;
                  break;
              case BT::NodeStatus::IDLE:
                  status = TreeStatus::IDLE;
                  break;
              case BT::NodeStatus::RUNNING:
                  status = TreeStatus::RUNNING;
                  break;
              case BT::NodeStatus::SUCCESS:
                  status = TreeStatus::SUCCESS;
                  break;
              case BT::NodeStatus::SKIPPED:
                  status = TreeStatus::SKIPPED;
                  break;
              /*case BT::NodeStatus::PAUSED:
                  status = TreeStatus::PAUSED;
                  break;*/
          }

          return status;
      };

      BT::NodeStatus bt_tree_status = GetTreeStatus();

      //Fill Service Tree info
      tree_status_msg.status = to_msg_status(bt_tree_status);
      tree_status_msg.name = tree_name_;
      tree_status_msg.file = tree_filename_;
      tree_status_msg.time_start = start_execution_time_;
      tree_status_msg.time = node_->get_clock()->now();
      tree_status_msg.uid = tree_uid_;
      tree_status_msg.details = execution_tree_error_;

      return tree_status_msg;
  }

  // Update and publishes atomically and mutually exclusive the current execution status.
  void TreeWrapper::UpdatePublishTreeExecutionStatus(const BT::NodeStatus status, const bool avoid_duplicate)
  {
      {
          std::lock_guard<std::mutex> lk(status_lock_);
          bool duplicate = status == status_;
          status_ = status;
          if(duplicate && avoid_duplicate) return; //already published
      }
      PublishExecutionStatus();
  }


void TreeWrapper::PublishExecutionStatus(bool error, std::string error_data)
{
  TreeStatus status_msg = buildTreeExecutionStatus();

    if (error)
    {
        status_msg.status    = TreeStatus::CRASHED;
        status_msg.details      = error_data;
    }
    bt_execution_status_publisher_->publish(status_msg);
}

void TreeWrapper::InitializeStatusPublisher()
{
    bt_transition_publisher_ = node_->create_publisher<Transition>("/"+tree_name_+"/transition_status", 1);
    bt_execution_status_publisher_ = node_->create_publisher<TreeStatus>("/"+tree_name_+"/execution_status", 100);
}

void TreeWrapper::SyncBlackboardUpdateCallback(const std::vector<BBEntry>& _bulk_upd)
  {
      for(const auto& upd : _bulk_upd)
          SyncBlackboardUpdateCallback(upd);
  }

  void TreeWrapper::SyncBlackboardUpdateCallback(const BBEntry& _single_upd)
  {
    //TODO:
    // - Missing BLACKBOARD::ENTRY-->ISSYNC() method
    // - Missing BLACKBOARD-->SET and BLACKBOARD-->SETANY
    /*  if(!is_tree_loaded_) return;

      // std::cout << "[BTWrapper "<<tree_identifier_<<"]::SyncBlackboardUpdateCallback " << 
      //     "\tkey=" << _single_upd.key << 
      //     "\ttype=" << _single_upd.type << 
      //     "\tvalue=" << _single_upd.value << "\n" << std::flush;
      // bool update_successful = false;
      
      const bool void_type = (_single_upd.type == BT::demangle(typeid(void))); // source tree does not know the type of the value

      //Get the Entry
      const BT::Blackboard::Entry* entry_ptr = global_blackboard_->getEntry(_single_upd.key).get();

     //retrieve string converter functor
      const BT::StringConverter* string_converter_ptr = (void_type || !entry_ptr)? nullptr : &entry_ptr->string_converter;
      
      //check string converter functor
      if(!void_type && string_converter_ptr == nullptr)
      {
          RCLCPP_ERROR(node_->get_logger(),"[BTWrapper %s] Entry in Sync. BB for key [%s] has type [%s], but no string converter can be found for this type", 
              tree_name_.c_str(), _single_upd.key.c_str(), _single_upd.type.c_str());
          return;
      }

      //retrieve current entry in bt server bb
      if(entry_ptr && entry_ptr->isSync())
      {
          // if(entry_ptr->port_info.missingTypeInfo()) is it necessary??? I would not update type info if received from another tree (i.e. from void to type T, with T != void)
          // {
          //     BT::Optional<BT::PortInfo> port_info_opt = bt_factory_ptr->getPortInfo(_single_upd.type);
          //     if(!port_info_opt.has_value())
          //     {
          //         ROS_ERROR("[BTWrapper %s] Entry in Sync. BB for key [%s] has type [%s], but it is an unknown type and therefore cannot be treated", tree_identifier_.c_str(), _single_upd.key.c_str(), _single_upd.type.c_str());
          //         return; // type unknown
          //     }
          //     tree_->rootBlackboard()->setPortInfo(_single_upd.key, port_info_opt.value());
          // }            
          //Get the TypeInfo
          auto type_info = global_blackboard_->entryInfo(_single_upd.key);
          if( BT::missingTypeInfo(type_info->type())  && !void_type && _single_upd.type != type_info->typeName()) //TODO evaluate strictness and checks to be made here
          {
              RCLCPP_ERROR(node_->get_logger(),"[BTWrapper %s]. Entry in Sync. BB for key [%s] has type [%s], but receiving requests for update with type [%s]",
                  tree_name_.c_str(), _single_upd.key.c_str(), type_info->typeName().c_str(), _single_upd.type.c_str());
              return; // type inconsistencies, don't update
          }
          
          try
          {
              if(!void_type)
              {
                  // convert from string new value
                  BT::Any new_any_value = type_info->parseString(_single_upd.value);
                  
                  // std::cout << "[BTWrapper "<<tree_identifier_<<"]::SyncBlackboardUpdateCallback built new_any_value with type " << BT::demangle(new_any_value.type()) << " \n" << std::flush;
                  // update it into the sync BB
                  global_blackboard_->setAny(_single_upd.key, std::move(new_any_value), true);
              }
              else
                  global_blackboard_->set(_single_upd.key, _single_upd.value, true);
          
          }
          catch(const std::exception& e)
          {
              std::cerr << "[BTWrapper "<<tree_name_<<"]::SyncBlackboardUpdateCallback fail to update value in BB for key [" << _single_upd.key << "]: " << e.what() << " \n" << std::flush;
              return;
          }

          // std::cout << "[BTWrapper "<<tree_identifier_<<"]::SyncBlackboardUpdateCallback updated value in BB for key [" << _single_upd.key << "] \n" << std::flush;
          // update_successful = true;
      }*/
  }

  void TreeWrapper::ResetLoggers()
  {
    loggers_init_ = false;
    bt_logger_cout_.reset();
    bt_logger_trace_.reset();
    bt_logger_file_.reset();
    //bt_logger_transition_rostopic_.reset();
    bt_logger_zmq_.reset();
  }

  void TreeWrapper::InitializeLoggers()
  {
    //Create Loggers
    const char* home = getenv("HOME");
    log_folder_ = log_folder_.front() == '~' ? std::string(home) + log_folder_.substr(1, log_folder_.size() - 1) : log_folder_;
    log_folder_ = log_folder_.back() == '/' ? log_folder_ : log_folder_ + "/";
    std::stringstream file_base;
    const auto& current_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    file_base << log_folder_ << "behavior_tree_node-" << tree_name_ << "_tree-" << std::put_time(std::localtime(&current_time), "%F-%R");

    if (enable_file_log_)
    {
      const auto& log_file       = file_base.str() + ".fbl";
      bt_logger_file_ = std::make_unique<BT::FileLogger>(tree_, log_file.c_str(), 20, true);
    }
    if (enable_minitrace_log_)
    {
      const auto& minitrace_file = file_base.str() + ".json";
      try
      {
          bt_logger_trace_ = std::make_unique<BT::MinitraceLogger>(tree_, minitrace_file.c_str());
      }
      catch(const BT::LogicError& ex)
      {
          RCLCPP_WARN(node_->get_logger(),"Error initializing Minitrace logger for %s: %s", tree_name_.c_str(), ex.what());
      }
    }
    if (enable_cout_log_)
    {
      try
      {
          bt_logger_cout_ = std::make_unique<BT::StdCoutLogger>(tree_);
      }
      catch(const BT::LogicError& ex)
      {
          RCLCPP_WARN(node_->get_logger(),"Error initializing Cout logger for %s: %s", tree_name_.c_str(), ex.what());
      }
    }
    if(enable_rostopic_log_)
    {
      //TODO:
     //bt_logger_transition_rostopic_ = std::make_unique<BT_ROS::RosTopicTransitionLogger>(tree_, bt_transition_publisher_);
    }

    if (enable_zmq_log_) 
    {
      bt_logger_zmq_ = std::make_unique<BT::PublisherZMQ>(tree_, 25, tree_publisher_port_,tree_server_port_);
      //TODO:
      //InitGrootV2Publisher();
    }

    loggers_init_ = true;
  }

  void TreeWrapper::LoadAllPlugins()
  {
      RCLCPP_INFO(node_->get_logger(),"LOADING PLUGINS");
      LoadPluginsFromROS(ros_plugin_directories_);
      LoadPluginsFromFolder();
      RCLCPP_INFO(node_->get_logger(),"LOADED PLUGINS");

  }

  void TreeWrapper::LoadPluginsFromFolder()
  {
    bool import_from_folder = false;
    node_->get_parameter_or("import_from_folder",import_from_folder,false);

    if(import_from_folder)
    {
        RCLCPP_INFO(node_->get_logger(),"LOADING PLUGINS FROM FOLDER");
        std::string plugins_folder;
        if (!node_->get_parameter("plugins_folder",plugins_folder))
        {
            RCLCPP_WARN(node_->get_logger(),"Import from folder option is set, but folder param is missing");
        }
        else
        {
            using namespace boost::filesystem;

            if(!exists(plugins_folder))
            {
              RCLCPP_ERROR(node_->get_logger(),"Plugin folder %s does not exist.", plugins_folder.c_str());
              return;
            }

            auto directory_list = [&] { return boost::make_iterator_range(directory_iterator(plugins_folder), {}); };

            for(const auto& entry : directory_list())
            {
                if((!is_regular_file(entry) && !is_symlink(entry)) || entry.path().extension() != ".so") { continue; }

                try
                {
                  const auto& plugin_path = canonical(entry.path());

                  factory_.registerFromPlugin(plugin_path.string());
                  loaded_plugins_.emplace(plugin_path.string());
                  RCLCPP_INFO(node_->get_logger(),"Loaded plugin %s from folder %s", plugin_path.filename().string().c_str(), plugins_folder.c_str());
                }
                catch(const std::runtime_error& ex)
                {
                  RCLCPP_ERROR(node_->get_logger(),"Cannot load plugin %s from folder %s. Error: %s", entry.path().filename().string().c_str(), plugins_folder.c_str(), ex.what());
                }
            }
        }
        RCLCPP_INFO(node_->get_logger(),"LOADING PLUGINS FROM FOLDER OK");
    }
  }

  void TreeWrapper::LoadPluginsFromROS(std::vector<std::string> ros_plugins_folders)
  {
      RCLCPP_INFO(node_->get_logger(),"LOADING PLUGINS FROM ROS");

      bt_server::Params bt_params;
      bt_params.ros_plugins_timeout = 1000;
      bt_params.plugins = ros_plugins_folders;
      RegisterPlugins(bt_params, factory_, node_);
      for(const auto& plugin : bt_params.plugins)
      {
        //RCLCPP_INFO(node_->get_logger(),"Added directory %s",plugin.c_str());
        loaded_plugins_.emplace(plugin);
      }
      RCLCPP_INFO(node_->get_logger(),"LOADING PLUGINS FROM ROS OK");
  }
  
  void TreeWrapper::InitializeBlackboard()
  {
    RCLCPP_INFO(node_->get_logger(),"CREATING BB");
    if (tree_bb_init_.size() > 0)
    {
      for(const auto& bb_init_abs_filepath: tree_bb_init_)
      {
          if(bb_init_abs_filepath.length() < 3) continue;
          InitializeBlackboardFile(bb_init_abs_filepath, false);
      }
    }
    RCLCPP_INFO(node_->get_logger(),"CREATING BB OK");
  }
  
  void TreeWrapper::InitializeBlackboardFile(const std::string& abs_file_path, const bool sync_bb)
  {
    //TODO:
    // - Missing BLACKBOARD-->SET
    // - Missing BLACKBOARD-->replaceKeysWithStringValues
    /*try 
    {
        // ROS_INFO("Initializing BB from YAML file %s", abs_file_path.c_str());
        YAML::Node config = YAML::LoadFile(abs_file_path);
        for(YAML::const_iterator it=config.begin();it!=config.end();++it)
        {
            const std::string& bb_key = it->first.as<std::string>();
            std::string bb_val = it->second.as<std::string>();
            //TODO:
            
            const BT::Optional<std::string> bbentry_value_inferred_keyvalues = blackboard_ptr->replaceKeysWithStringValues(bb_val, true); // no effect if it has no key
            if(!bbentry_value_inferred_keyvalues)
            {
                // but will complain if it has a reference to a wrong key
                RCLCPP_ERROR(node_->get_logger(),"Init. of BB key %s for value %s, value inference did not succeed: %s", 
                    bb_key.c_str(), 
                    bb_val.c_str(),
                    bbentry_value_inferred_keyvalues.error().c_str());
                continue; // and skip this init
            }
            else
                bb_val = bbentry_value_inferred_keyvalues.value();

            RCLCPP_INFO(node_->get_logger(),"Init. BB key [\"%s\"] with value \"%s\"", bb_key.c_str(), bb_val.c_str());
            // use the string here and blackboard_ptr->set(...)
            blackboard_ptr->set(bb_key, bb_val, sync_bb);
        }
        RCLCPP_INFO(node_->get_logger(),"Initialized BB with %ld entries from YAML file %s", global_blackboard_->getKeys().size(), abs_file_path.c_str());
    }
    catch(const YAML::Exception& ex) 
    { 
        RCLCPP_ERROR(node_->get_logger(),"Initializing. BB key from file '%s' did not succeed: %s", abs_file_path.c_str(), ex.what());
    }*/
  }

  void TreeWrapper::CreateTree(const std::string& full_path)
  {
    tree_ = factory_.createTreeFromFile(full_path,global_blackboard_);
  }
}  // namespace BT
