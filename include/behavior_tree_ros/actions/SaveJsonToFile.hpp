#ifndef SAVE_JSON_TO_FILE_HPP
#define SAVE_JSON_TO_FILE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include "behavior_tree_ros/details/conversion_json.hpp"
#include <ros/package.h>
#include <fstream>
#include <boost/filesystem.hpp>

namespace BT_ROS
{
class SaveJsonToFile final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~SaveJsonToFile() = default;

        static BT::PortsList providedPorts() {
            return { 
                BT::InputPort<std::string>("file_name", "Name of the file"),
                BT::InputPort<std::string>("prefix", "Prefix path of the file"),
                BT::InputPort<nlohmann::json>("json_object", "Parameter to save as json")
            };
        }

        virtual BT::NodeStatus tick() override {
            const auto& file_name  = getInput<std::string>("file_name");
            if(!file_name)  { throw BT::RuntimeError { name() + ": " + file_name.error()  }; }
            const auto& prefix  = getInput<std::string>("prefix");
            if(!prefix)  { throw BT::RuntimeError { name() + ": " + prefix.error()  }; }
            const auto& json_object  = getInput<nlohmann::json>("json_object");
            if(!json_object)  { throw BT::RuntimeError { name() + ": " + json_object.error()  }; }

            const char *homedir;
            if ((homedir = getenv("HOME")) == NULL) {
                homedir = getpwuid(getuid())->pw_dir;
            }


            
            std::string path_ = std::string(homedir) + "/.bt_ros/"; // + prefix.value();
            boost::filesystem::path dir(path_);
            if(!(boost::filesystem::exists(dir))){
                if ( !boost::filesystem::create_directory(dir) )
                    return BT::NodeStatus::FAILURE;
            }

            path_ += prefix.value();
            boost::filesystem::create_directories(path_);


            //check if the path exist
            std::string filename = path_ + "/" + file_name.value() + ".json";
            std::cout << "FILENAME: " << filename << std::endl;

            std::ofstream file(filename);
            if (!file.is_open()) return BT::NodeStatus::FAILURE;

            //TODO: a good feature would be to parse the json file 
            file << json_object.value();
            return BT::NodeStatus::SUCCESS;
       
        }
};
}

#endif
