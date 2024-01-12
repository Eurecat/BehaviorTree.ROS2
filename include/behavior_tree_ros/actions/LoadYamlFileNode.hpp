#ifndef LOAD_YAML_FILE_NODE_HPP
#define LOAD_YAML_FILE_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include "yaml-cpp/yaml.h"
#include "behavior_tree_ros/details/conversion_json.hpp"
#include <pwd.h>
#include <ros/package.h>

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
                     BT::OutputPort<nlohmann::json>("output", "Parameter list as json")
                    //  BT::OutputPort<std::string>("output", "Parameter list as json")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& file_path  = getInput<std::string>("file_path");
            if(!file_path)  { throw BT::RuntimeError { name() + ": " + file_path.error()  }; }
            
            std::string absolute_file_path = file_path.value();

            //1. Check for a ROS PATH
            std::size_t found = absolute_file_path.find("$(find ");
            if (found!=std::string::npos)
            {
                std::size_t end_package_pos = absolute_file_path.find(")");
                std::string package_name = absolute_file_path.substr (7,(end_package_pos-7));
                std::string package_relative_path = absolute_file_path.substr (end_package_pos+1); 
                std::string ros_pkg_path = ros::package::getPath(package_name);
                absolute_file_path = ros_pkg_path + package_relative_path;
            }
            //2. Check for a Relative Path to HOME and get the absolute one
            else if(file_path.value()[0] == '~')
            {
                absolute_file_path.erase(0, 1); // remove '~'
                
                const char *homedir;
                if ((homedir = getenv("HOME")) == NULL)
                    homedir = getpwuid(getuid())->pw_dir;

                absolute_file_path = std::string(homedir) + absolute_file_path;
            }

            // TODO: Add support for more complex YAML files
            // Load YAML File
            // std::cout << std::endl << "--> Loading Yaml File " << absolute_file_path << std::endl;
            try {
                YAML::Node config = YAML::LoadFile(absolute_file_path);

                // Parse to JSON
                nlohmann::json json;
                for(YAML::const_iterator it=config.begin();it!=config.end();++it)
                {
                    // std::cout << "    - " << it->first.as<std::string>() << " = "
                    //                     << it->second.as<std::string>() << std::endl;
                    json[it->first.as<std::string>()] = it->second.as<std::string>();
                }

                // Return json as result
                setOutput("output", json);
                return BT::NodeStatus::SUCCESS;
            }
            catch(const YAML::Exception&) { return BT::NodeStatus::FAILURE; }
        }
};
}

#endif
