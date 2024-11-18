#include "tree_wrapper.hpp"

#include "yaml-cpp/yaml.h"

namespace BT
{

  TreeWrapper::TreeWrapper(const rclcpp::Node::SharedPtr& node)
    : node_(node)
  {
    global_blackboard_ = BT::Blackboard::create();
  }

  TreeWrapper::~TreeWrapper()
  {}

  bool TreeWrapper::ResetTree()
  {
      if(IsTreeLoaded() )
      {
          tree_.haltTree();
          is_tree_loaded_ = false;
          return true;
      }
      else
          return false;
  }

  void TreeWrapper::RemoveTree()
  {
     ResetTree();
  }

  void TreeWrapper::InitGrootPublisher()
  {
    //NEW GROOT
    groot_publisher_.reset();
    groot_publisher_ = std::make_shared<BT::Groot2Publisher>(tree_, tree_server_port_);
    //OLD GROOT

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
      for(const auto& plugin : bt_params.plugins)
      {
        RCLCPP_INFO(node_->get_logger(),"Added directory %s",plugin.c_str());
      }
      RegisterPlugins(bt_params, factory_, node_);

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
    try 
    {
        // ROS_INFO("Initializing BB from YAML file %s", abs_file_path.c_str());
        YAML::Node config = YAML::LoadFile(abs_file_path);
        for(YAML::const_iterator it=config.begin();it!=config.end();++it)
        {
            const std::string& bb_key = it->first.as<std::string>();
            std::string bb_val = it->second.as<std::string>();
            //TODO:
            /*
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
            blackboard_ptr->set(bb_key, bb_val, sync_bb);*/
        }
        RCLCPP_INFO(node_->get_logger(),"Initialized BB with %ld entries from YAML file %s", global_blackboard_->getKeys().size(), abs_file_path.c_str());
    }
    catch(const YAML::Exception& ex) 
    { 
        RCLCPP_ERROR(node_->get_logger(),"Initializing. BB key from file '%s' did not succeed: %s", abs_file_path.c_str(), ex.what());
    }
  }

  void TreeWrapper::CreateTree(const std::string& full_path)
  {
    tree_ = factory_.createTreeFromFile(full_path,global_blackboard_);
  }
}  // namespace BT
