[![build](https://github.com/Eurecat/BehaviorTree.ROS2/actions/workflows/build.yml/badge.svg)](https://github.com/Eurecat/BehaviorTree.ROS2/actions/workflows/build.yml)

# behaviortree_ros2

## Scope
This repository contains a ROS 2 layer for wrapping the functionalities of [BehaviorTree.CPP](https://www.behaviortree.dev/) . In particular, it provides a standard way to implement:

-   Behavior Tree Executor with ROS 2 Action interface
-   Action clients
-   Service Clients
-   Topic Subscribers
-   Topic Publishers






## Use behaviortree_ros2
To use this repository you can rely on [Docker](https://www.docker.com/) framework. In the source you will find convenient Dockerfile to compile the image and the docker-compose to transform the image into a container. 

To install the docker framework, you should install docker on your system: [follow this guide](https://docs.docker.com/engine/install/ubuntu/). 

After docker, use the following commands

	$ git@github.com:Eurecat/behaviortree_ros2.git 
	$ cd behaviortree_ros2
	$ ./scripts/install.sh

Now the image is correctly compiled and installed in your system. You can run it with the following one	

	$ ./scripts/compose.sh

This last command will setup your workspace. First, it will download the needed dependencies:
__TODO__: add deps


First example

	$ ros2 launch bt_examples executor.launch.xml 
	$ ros2 action send_goal /bt_action_server_example btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: ExampleTree1}"



# BehaviorTree.ROS2
<!-- [![Test](https://github.com/BehaviorTree/BehaviorTree.ROS2/actions/workflows/test.yml/badge.svg)](https://github.com/BehaviorTree/BehaviorTree.ROS2/actions/workflows/test.yml) -->

This repository contains useful wrappers to use ROS2 and BehaviorTree.CPP together.
In particular, it provides a standard way to implement:

- Behavior Tree Executor with ROS Action interface.
- Action clients.
- Service Clients.
- Topic Subscribers.
- Topic Publishers.

Our main goals are:

- to minimize the amount of boilerplate.
- to make asynchronous Actions non-blocking.

It uses [behaviortree_eut_plugins](https://ice.eurecat.org/gitlab/robotics-automation/behavior_tree_eut_plugins) for augmenting its original [public version](https://github.com/BehaviorTree/BehaviorTree.ROS2.git) with custom serialization and deserialization policies, providing out of the box json one, essentially adding the following plugins templates to be used with a one-liner:

Plugin Template | Deserialization Policy | Serialization Policy | Extra Pre/Post processing 
--- | --- | --- | ---  
Publisher | NoDeserialization | - | -
AutoDesPublisher | AutomaticDeserialization | - | -
| | |
Subscriber | - | NoSerialization | -
AutoSerSubscriber | - | AutomaticSerialization | - 
JsonSerSubscriber | - | JsonSerialization | - 
SmartJsonSerSubscriber | - | SmartJsonSerialization | &check;
SmartTimeEnabledJsonSerSubscriber | - | SmartJsonSerialization | &check;
| | |
ServiceClient | NoDeserialization | NoSerialization | -
AutoDesServiceClient | AutomaticDeserialization | NoSerialization | - 
JsonSerServiceClient | NoDeserialization | JsonSerialization | - 
AutoDesJsonSerServiceClient | AutomaticDeserialization | JsonSerialization | - 
AutoDesAutoSerServiceClient | AutomaticDeserialization | AutomaticSerialization | - 
AutoDesSmartJsonSerServiceClient | AutomaticDeserialization | SmartJsonSerialization | &check;
| | |
ActionClient | NoDeserialization | NoSerialization | -
AutoDesJsonSerActionClient | AutomaticDeserialization | JsonSerialization | - 
AutoDesAutoSerActionClient | AutomaticDeserialization | AutomaticSerialization | - 
AutoDesSmartJsonSerActionClient | AutomaticDeserialization | SmartJsonSerialization | &check;

Explained in very simple terms these are how the aforementioned serialization / deserialization policies operate:

- **NoDeserialization** would treat the msg of type `T` with type `T`
- **NoSerialization** would treat the msg of type `T` with type `T` 
- **AutomaticDeserialization** would explode the msg of type `T` with a port for each simple and raw type (e.g. uint8,.. uint64, int8, .., int64, bool, float, double, string ), except time related ones.
- **JsonSerialization** would serialize the msg of type `T` into a `nlohmann::json` 
- **SmartJsonSerialization** would serialize the msg of type `T` into a `nlohmann::json` and offers additional pre/post processing utilities, specifically:
    - `ignore_fields` port to specify the `std::vector<std::String>` that shall be ignored during serialization avoiding not needed data processing and storage
    -  possibilities of specialize template functions for pre or post processing of the message

        - `void useMsgBeforeSerialization(const std::shared_ptr<T> _message, BT::TreeNode& _tree_node)`
        - `void processMsgPostSerialization(const std::shared_ptr<T> _message, nlohmann::json& json, BT::TreeNode& _tree_node)`


## How to use

### Sample plugin.cpp
To create a new plugin, you just need to write a .cpp file in your ros2 behaviortree_ros2 dependent package, include the msg, srv, action definitions that you are interested in a write simple one liners such as:

```
#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/serialized_sub_node.hpp"
#include "behaviortree_ros2/serialized_pub_node.hpp"
#include "behaviortree_ros2/serialized_srv_node.hpp"
#include "behaviortree_ros2/serialized_act_node.hpp"

#include <std_msgs/msg/empty.hpp>

#include <std_srvs/srv/empty.hpp>

#include "btcpp_ros2_interfaces/action/sleep.hpp"

using namespace BT;


BT_REGISTER_ROS_NODES(factory, params)
{
    //SUBSCRIBERS
    factory.registerNodeType<SmartSerializedSubscriber<std_msgs::msg::Empty>>("MonitorStdEmpty",params);
    
    //PUBLISHERS
    factory.registerNodeType<AutoDesPublisher<std_msgs::msg::Empty>>("PublishStdEmpty",params);
    
    // SERVICE CLIENTS
    factory.registerNodeType<AutomaticServiceClient<std_srvs::srv::Empty>>("CallEmptyService",params);
    
    // ACTION CLIENTS
    factory.registerNodeType<AutomaticSimpleActionClient<btcpp_ros2_interfaces::action::Sleep>>("TestActionSleep",params);

};
```

As you can see, creating a new action client is as simple as adding a line, without avoiding repetition of redundant boiler plate code.

Note that you can specialize some functions for specific MessageType `T` in the case in which you want to deviate from the "standard" of ports and respective types imposed by the serialization or deserialization msg. For instance, ...

```
template<>
BT::PortsList 
AutomaticDeserialization<T>::requiredPorts()
{
        return { BT::InputPort<std::string>("x", ""), ... };
}

template<>
T
AutomaticDeserialization<T>::buildMessage(const BT::TreeNode& _tree_node)
{
        T msg {};
        // ...
        return msg;
}
```

### From plugin.cpp to .so and .xml

To generate a .so of plugins to be imported at run time, you need to add this into the CMakeLists.txt of your package (being `src/plugin.cpp` a file as the one above calling `BT_REGISTER_ROS_NODES`):

```
# Build project plugin library .so
add_library(${PROJECT_NAME}_nodemodels SHARED src/plugin.cpp)
target_include_directories(${PROJECT_NAME}_nodemodels PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_definitions(${PROJECT_NAME}_nodemodels PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(${PROJECT_NAME}_nodemodels ${THIS_PACKAGE_DEPS})

...

install(TARGETS
  ${PROJECT_NAME}_nodemodels
  LIBRARY DESTINATION share/${PROJECT_NAME}/${PLUGINS_FOLDER_PATH}
  ARCHIVE DESTINATION share/${PROJECT_NAME}/${PLUGINS_FOLDER_PATH}
  RUNTIME DESTINATION share/${PROJECT_NAME}/${PLUGINS_FOLDER_PATH}
  )
```

For further generating the xml at compilation time and make sure they end up in your desired path, I would suggest you to examine the following files in behaviortree_ros2 pkg to serve as example:

- [plugins.cpp](behaviortree_ros2/src/plugin.cpp): specifying nodemodels to be created with btros2 and packed into a .so
- descriptors:
  - [plugins_description.xml](behaviortree_ros2/plugins_description.xml): listing the folder relative path in share for the ros2 pkg where to store the compiled xml version of the automatically generated nodemodels at compile time and packed in the .so
  - [palettes_description.xml](behaviortree_ros2/palettes_description.xml): listing the folder relative path in share for the ros2 pkg where to store the xml for relevant reusable subtrees
- [CMakelists.txt](behaviortree_ros2/CMakeLists.txt): take a closer look to how, after the lines to generate the .so, we recover from the xml descriptors the folder path and we call the script bt_ros_plugins_to_xml to generate and install the xml for the nodemodels already at compile time.

# Examples

I will encourage you to look at its usage in the [BT workshop for ROSCON ES 2025](https://github.com/Eurecat/BTWorkshop_ROSCON_ES25).

# Further Documentation

- [ROS Behavior Wrappers](behaviortree_ros2/ros_behavior_wrappers.md)
- [TreeExecutionServer](behaviortree_ros2/tree_execution_server.md)
- [Sample Behaviors](btcpp_ros2_samples/README.md)

Note that this library is compatible **only** with:

- **BT.CPP** 4.6 or newer.
- **ROS2** Humble or Jazzy.


## Acknowledgements

A lot of code is either inspired or copied from [Nav2](https://docs.nav2.org/).

For this reason, we retain the same license and copyright.