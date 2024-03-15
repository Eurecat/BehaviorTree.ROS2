#ifndef LOAD_JSON_FROM_FILE_HPP
#define LOAD_JSON_FROM_FILE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include "behavior_tree_ros/details/conversion_json.hpp"
#include <ros/package.h>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>


using namespace std;

namespace BT_ROS
{
class LoadJsonFromFile final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~LoadJsonFromFile() = default;

        static BT::PortsList providedPorts()
        {
            return { 
                BT::InputPort<std::string>("file_name", "Name of the file"),
                BT::InputPort<std::string>("prefix", "Prefix path of the file"),
                BT::OutputPort<nlohmann::json>("json_object", "Parameter to load as json")
            };
        }

        virtual BT::NodeStatus tick() override {

            const auto& file_name  = getInput<std::string>("file_name");
            if(!file_name)  { throw BT::RuntimeError { name() + ": " + file_name.error()  }; }
            
            const auto& prefix  = getInput<std::string>("prefix");
            if(!prefix)  { throw BT::RuntimeError { name() + ": " + prefix.error()  }; }

            const char *homedir;
            if ((homedir = getenv("HOME")) == NULL) {
                homedir = getpwuid(getuid())->pw_dir;
            }

            //Default path in the home folder: ${HOME}/.bt_ros/file
            std::string filename = std::string(homedir) + "/.bt_ros/" + prefix.value() + "/" + file_name.value() + ".json";
            std::ifstream file(filename);
            if (  !file.is_open() ) { 
                std::cerr << "Impossible to open the json file" << endl; 
                return BT::NodeStatus::FAILURE;
            } //Error: impossible to open the file

            try {
                // parsing input with a syntax error
                nlohmann::json json_object = nlohmann::json::parse(file);
                setOutput("json_object", json_object);
                return BT::NodeStatus::SUCCESS;
            } //Success
            catch (const nlohmann::json::parse_error& e) {
                // output exception information
                std::cout << "message: " << e.what() << '\n'
                        << "exception id: " << e.id << '\n'
                        << "byte position of error: " << e.byte << std::endl;
                return BT::NodeStatus::FAILURE;
            }//Error: exception reading the json file
        }
};
}

#endif
