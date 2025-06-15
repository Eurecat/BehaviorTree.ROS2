[![build](https://github.com/Eurecat/behaviortree_ros2/actions/workflows/build.yml/badge.svg)](https://github.com/Eurecat/behaviortree_ros2/actions/workflows/build.yml)


# BehaviorTree.ROS2


## Scope
This repository contains a ROS 2 layer for wrapping the functionalities of [BehaviorTree.CPP](https://www.behaviortree.dev/).

Using this package you will have an executor for ROS 2 behavior trees and different nodes already prepared to interface with the ROS 2 system, like node publisher subscribers, services and actions. 

## Use BehaviorTree.ROS2

- Dockers is used to simplify the installation and the deployment of  BehaviorTree.ROS2. To install the dockers [follow this guide](https://docs.docker.com/engine/install/ubuntu/). 

- Your public ssh keys should be correcly installed on the github portal to succesfully download the dependencies using the docker container. 

- To download and start the BT executore, move in the desired development folder and use the following command.

    	$ git clone git@github.com:Eurecat/BehaviorTree.ROS2.git
	    $ cd BehaviorTree.ROS2/
	    $ ./scripts/install.sh && ./scripts/compose.sh

    __Note that, the first time you run this command it could take a more time than expected since it creates the image and compile the workspace with the BehaviorTree.ROS2 dependencies.__

- You can log into the contenier with one or more terminal using the following script.
	
        $ ./scripts/exec.sh



## Test the  BehaviorTree.ROS2

To check that everything is installed correctly, you can test the following example. 

- Start the executor

        $ ros2 launch bt_examples executor.launch.xml 
	
- Request to start the tree called ExampleTree1
    
        $ ros2 action send_goal /bt_action_server_example btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: ExampleTree1}"

- Monitor the execution status using groot

        $ ros2 run groot Groot

    __After opened groot, you must click on connect to start the monitoring__




### TODO: discuss a better example
### TODO: document the nodes implemented in the plugins
### TODO: describe how to implement new plugins


### Dependencies


// TODO

Note that you can specialize some functions for specific MessageType `T` in the case in which you want to deviate from the "standard" of ports and respective types imposed by the serialization or deserialization msg. For instance, ...

//TODO

# Examples

# Further Documentation

- [ROS Behavior Wrappers](behaviortree_ros2/ros_behavior_wrappers.md)
- [TreeExecutionServer](behaviortree_ros2/tree_execution_server.md)
- [Sample Behaviors](btcpp_ros2_samples/README.md)

Note that this library is compatible **only** with:

- **BT.CPP** 4.6 or newer.
- **ROS2** Humble or Jazzy.

Additionally, check **plugins.hpp** and **plugins.cpp** to see how to learn how to
wrap your Nodes into plugins that can be loaded at run-time.


## Acknowledgements

A lot of code is either inspired or copied from [Nav2](https://docs.nav2.org/).

For this reason, we retain the same license and copyright.
