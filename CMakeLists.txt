cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(behavior_tree_ros VERSION 0.4.1)

set(CMAKE_CXX_STANDARD 14)

if($ENV{ROS_DISTRO} STREQUAL "melodic" )
add_definitions(-DMELODIC)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        message_generation
        ros_type_introspection
        topic_tools
        actionlib
        behavior_tree_core
        cmake_modules
        tf
        angles
)
find_package(behavior_tree_core 3.2.0 REQUIRED)

find_package(TinyXML2 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_service_files(
    FILES
        GetLoadedPlugins.srv
        LoadTree.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  BehaviorTree.action
)

generate_messages(DEPENDENCIES
  actionlib_msgs  std_msgs)

catkin_package(
    CATKIN_DEPENDS 
        roscpp
        message_runtime
        ros_type_introspection
        behavior_tree_core
        topic_tools
        actionlib
        cmake_modules
        tf
        angles
    DEPENDS
	    TinyXML2
    INCLUDE_DIRS
        include
)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} INTERFACE ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME} INTERFACE -Wall -Werror -Wfatal-errors -Wl,--no-undefined)

add_library(${PROJECT_NAME}_plugin src/plugin.cpp src/details/conversion_json.cpp)
target_link_libraries(${PROJECT_NAME}_plugin ${catkin_LIBRARIES} yaml-cpp)
target_include_directories(${PROJECT_NAME}_plugin PUBLIC include PRIVATE ${catkin_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME}_plugin PRIVATE -Wall -Werror -Wfatal-errors -Wl,--no-undefined)
target_compile_definitions(${PROJECT_NAME}_plugin PRIVATE BT_PLUGIN_EXPORT)
add_dependencies(${PROJECT_NAME}_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS})
list(APPEND ${PROJECT_NAME}_targets ${PROJECT_NAME}_plugin)

add_executable(${PROJECT_NAME}_node src/behavior_tree_node.cpp src/BehaviorTreeNode.cpp src/bt_rostopic_logger.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE include ${catkin_INCLUDE_DIRS} ${TinyXML2_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES})
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Werror -Wfatal-errors -Wl,--no-undefined)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
list(APPEND ${PROJECT_NAME}_targets ${PROJECT_NAME}_node)

add_executable(${PROJECT_NAME}_action_node src/behavior_tree_action_node.cpp src/BehaviorTreeActionNode.cpp src/bt_rostopic_logger.cpp)
target_include_directories(${PROJECT_NAME}_action_node PRIVATE include ${catkin_INCLUDE_DIRS} ${TinyXML2_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_action_node PRIVATE ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES})
target_compile_options(${PROJECT_NAME}_action_node PRIVATE -Wall -Werror -Wfatal-errors -Wl,--no-undefined)
add_dependencies(${PROJECT_NAME}_action_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
list(APPEND ${PROJECT_NAME}_targets ${PROJECT_NAME}_action_node)


install(TARGETS ${${PROJECT_NAME}_targets}
        ARCHIVE  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        INCLUDES DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
