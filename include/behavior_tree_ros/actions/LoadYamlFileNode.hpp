#ifndef LOAD_YAML_FILE_NODE_HPP
#define LOAD_YAML_FILE_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include "yaml-cpp/yaml.h"
#include "behavior_tree_ros/details/conversion_json.hpp"
#include <pwd.h>

namespace BT_ROS
{
class LoadYamlFileNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~LoadYamlFileNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<std::string>("file_path", "Path to the YAML config file"),
                     //BT::OutputPort<nlohmann::json>("output", "Parameter list as json")
                     BT::OutputPort<std::string>("output", "Parameter list as json")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);
            const auto& file_path  = getInput<std::string>("file_path");
            if(!file_path)  { throw BT::RuntimeError { name() + ": " + file_path.error()  }; }
            
            // Check if it is a relative path to HOME and get the absolute one
            std::string absolute_file_path = file_path.value();
            if(file_path.value()[0] == '~')
            {
                absolute_file_path.erase(0, 1); // remove '~'
                
                const char *homedir;
                if ((homedir = getenv("HOME")) == NULL)
                    homedir = getpwuid(getuid())->pw_dir;

                absolute_file_path = std::string(homedir) + absolute_file_path;
            }

            // Load YAML File
            std::cout << std::endl << "--> Loading Yaml File " << absolute_file_path << std::endl;
            YAML::Node config = YAML::LoadFile(absolute_file_path);
            
            // Parse to JSON
            nlohmann::json json;
            for(YAML::const_iterator it=config.begin();it!=config.end();++it) 
            {
                std::cout << "    - " << it->first.as<std::string>() << " = " 
                                      << it->second.as<std::string>() << std::endl;
                json[it->first.as<std::string>()] = it->second.as<std::string>(); 
            }
            
            // Return json as result
            setOutput("output", json);
            return BT::NodeStatus::SUCCESS;
        }
};
}

#endif
