#include <stdio.h>
#include <iostream>
#include <fstream>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/xml_parsing.h"

#include "behaviortree_ros2/plugins.hpp"

int main(int argc, char* argv[])
{

    if (argc != 3)
    {
        printf("Wrong number of command line arguments\nUsage: %s [filename_in.so] [filename_out.xml] \n", argv[0]);
        return 1;
    }

    std::string plugin_path = argv[1];

    BT::BehaviorTreeFactory factory;

    BT::SharedLibrary loader(plugin_path);
    if(loader.hasSymbol(BT::PLUGIN_SYMBOL))
    {
        typedef void (*Func)(BT::BehaviorTreeFactory&);
        auto func = (Func)loader.getSymbol(BT::PLUGIN_SYMBOL);
        func(factory);
    }
    else if (loader.hasSymbol(BT::ROS_PLUGIN_SYMBOL))
    {
        BT::RosNodeParams ros_params; // should not need actually instantiated ros params just to load models and translate them to xml
        // ros_params.nh = nullptr;
        // ros_params.server_timeout = std::chrono::milliseconds(1000);
        // ros_params.wait_for_server_timeout =  std::chrono::milliseconds(1000);
        typedef void (*Func)(BT::BehaviorTreeFactory&, const BT::RosNodeParams&);
        auto func = (Func)loader.getSymbol(BT::ROS_PLUGIN_SYMBOL);
        func(factory, ros_params);
    }
    else
    {
        printf("The file %s is not a valid BehaviorTree plugin\n", argv[1]);
        return 1;
    }

    std::string tree_xml = writeTreeNodesModelXML(factory,false);
 
    std::ofstream outfile;
    outfile.open(argv[2]);
    outfile << "<?xml version=\"1.0\"?>" << std::endl;
    outfile << tree_xml; 
    return 0;
}