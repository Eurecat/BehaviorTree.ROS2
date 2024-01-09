cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(behavior_tree_ros VERSION 0.6.3)

set(CMAKE_CXX_STANDARD 14)

execute_process(COMMAND cat "/etc/issue" OUTPUT_VARIABLE UBUNTU_VERSION)

if(${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER 6.0)
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fconcepts")
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
        # behavior_tree_core
        behaviortree_cpp_v3
        cmake_modules
        tf
        angles
)
# find_package(behavior_tree_core 3.2.0 REQUIRED)
find_package(behaviortree_cpp_v3 3.2.0 REQUIRED)
find_package(yaml-cpp REQUIRED)


## Generate actions in the 'action' folder
add_action_files(
  FILES
  BehaviorTree.action
  PerformHandShake.action
  ExchangeInfo.action
)

add_message_files(
    FILES
        HandShake.msg
        ExecutionStatus.msg
        TreeStatus.msg
)

add_service_files(
    FILES
        GetLoadedPlugins.srv
        LoadTree.srv
        GetTreeStatus.srv
)

generate_messages(DEPENDENCIES
  actionlib_msgs  std_msgs)

catkin_package(
    CATKIN_DEPENDS 
        roscpp
        message_runtime
        ros_type_introspection
        # behavior_tree_core
        behaviortree_cpp_v3
        topic_tools
        actionlib
        cmake_modules
        tf
        angles
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

add_executable(${PROJECT_NAME}_node src/behavior_tree_node.cpp src/BehaviorTreeNode.cpp src/bt_rostopic_logger.cpp
                                    src/3rdparty/tinyxml2/tinyxml2.cpp src/details/TreeWrapper.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${catkin_LIBRARIES} yaml-cpp)
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Werror -Wfatal-errors -Wl,--no-undefined)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
list(APPEND ${PROJECT_NAME}_targets ${PROJECT_NAME}_node)

add_executable(${PROJECT_NAME}_intercom_node src/behavior_tree_intercom_node.cpp src/BehaviorTreeIntercomNode.cpp )
target_include_directories(${PROJECT_NAME}_intercom_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_intercom_node PRIVATE ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_intercom_node PRIVATE -Wall -Werror -Wfatal-errors -Wl,--no-undefined)
add_dependencies(${PROJECT_NAME}_intercom_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
list(APPEND ${PROJECT_NAME}_targets ${PROJECT_NAME}_intercom_node)

install(TARGETS ${${PROJECT_NAME}_targets}
        ARCHIVE  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        INCLUDES DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
