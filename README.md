# behavior_tree_ros
![Build and Release for ROS Kinetic & ROS Melodic](https://github.com/robotics-upo/behavior_tree_ros/workflows/Build%20and%20Release%20for%20ROS%20Kinetic%20&%20ROS%20Melodic/badge.svg)

Provides a ROS wrapper for the Behavior Tree engine (BehaviorTree.CPP library), as well as ROS-based pluggins to deal with ROS topics, services and actions within a BT.


## ROS nodes

The repository provides a ROS node that allow to execute a given tree using the BT engine. Two different interfaces to the node are provided:

- A **service** interface: the node can be requested to execute a tree
- An [actionlib](http://wiki.ros.org/actionlib) interface: it implements a **SimpleActionServer** that permits to cancel trees, returns feedback, etc 

Furthermore, the node offers a new BT status logger option to monitor the status of the execution through a ROS topic


- [] Currently, two different nodes implement the service and the actionlib interfaces. Merge both into one



## ROS plugins for BehaviorTree.CPP

The repository also adds a plugin of new BT nodes (not to be confused with the ROS nodes) for BehaviorTree.CPP related to ROS functionalities. In particular, it adds actions nodes related to:

- **Subscriber** leaf nodes can serialize ROS messages they receive into a more general structure, allowing for more general-purpose smaller blocks capable of inspecting and performing operations on these messages. 
- **Pubisher** leaf nodes.
- **Services** leaf nodes to call ROS services
- **Actionlib** support for asynchronous actions using the **SimpleActionClient** model (see actionlib). This, together with the actionlib interface of the tree engine itself allows to encapsulate subtrees in separated ROS nodes as well.

These action nodes use ros-type-instrospection for automatic serialization of ROS messages, so that general BT nodes can be used to deal with different types of ROS messages.

It also adds some additional decorator nodes.

Please, see the documentation.


## Dependencies

* **ros-type-introspection**
```bash
sudo apt-get install ros-$ROS_DISTRO-ros-type-introspection
```

* **yaml-cpp**
```bash
sudo apt-get install libyaml-cpp-dev
```