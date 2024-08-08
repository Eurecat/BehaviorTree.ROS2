cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(behavior_tree_ros)

set(CMAKE_CXX_STANDARD 17)

execute_process(COMMAND cat "/etc/issue" OUTPUT_VARIABLE UBUNTU_VERSION)

if(${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER 6.0)
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fconcepts")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        message_generation
        ros_type_introspection
        topic_tools
        actionlib
        behavior_tree_cpp
        cmake_modules
        tf
        angles
)
# find_package(behavior_tree_core 3.2.0 REQUIRED)
find_package(behavior_tree_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)


find_package(ZMQ)
if( ZMQ_FOUND )
    add_definitions( -DBT_SERVER_SYNC )
    message(STATUS "ZeroMQ found.")
    add_definitions( -DZMQ_FOUND )
else()
    message(WARNING "ZeroMQ NOT found.")
endif()

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
        TreeExecutionStatus.msg
        Transition.msg
)

add_service_files(
    FILES
        GetLoadedPlugins.srv
        LoadTree.srv
        StopTree.srv
        GetTreeStatus.srv
        GetTreeStatusByID.srv
        GetAllTreesStatus.srv
)

generate_messages(DEPENDENCIES
  actionlib_msgs  std_msgs)

catkin_package(
    CATKIN_DEPENDS 
        roscpp
        message_runtime
        ros_type_introspection
        behavior_tree_cpp
        topic_tools
        actionlib
        cmake_modules
        tf
        angles
    INCLUDE_DIRS
        include
)

include_directories(include ${behavior_tree_cpp_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} INTERFACE ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME} INTERFACE -Wall -Werror -Wfatal-errors -Wl,--no-undefined)

add_library(${PROJECT_NAME}_plugin src/plugin.cpp src/details/conversion_json.cpp)
target_link_libraries(${PROJECT_NAME}_plugin ${catkin_LIBRARIES} yaml-cpp  ${Boost_LIBRARIES})
target_include_directories(${PROJECT_NAME}_plugin PUBLIC include PRIVATE ${catkin_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME}_plugin PRIVATE -Wall -Werror -Wfatal-errors -Wl,--no-undefined)
target_compile_definitions(${PROJECT_NAME}_plugin PRIVATE BT_PLUGIN_EXPORT)
add_dependencies(${PROJECT_NAME}_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS})
list(APPEND ${PROJECT_NAME}_targets ${PROJECT_NAME}_plugin)

add_executable(${PROJECT_NAME}_node src/behavior_tree_node.cpp src/BehaviorTreeNode.cpp src/bt_rostopic_logger.cpp
                                    src/3rdparty/tinyxml2/tinyxml2.cpp src/details/TreeWrapper.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${catkin_LIBRARIES} yaml-cpp ${Boost_LIBRARIES})
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Werror -Wfatal-errors -Wl,--no-undefined)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
list(APPEND ${PROJECT_NAME}_targets ${PROJECT_NAME}_node)

add_executable(${PROJECT_NAME}_intercom_node src/behavior_tree_intercom_node.cpp src/BehaviorTreeIntercomNode.cpp )
target_include_directories(${PROJECT_NAME}_intercom_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_intercom_node PRIVATE ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_intercom_node PRIVATE -Wall -Werror -Wfatal-errors -Wl,--no-undefined)
add_dependencies(${PROJECT_NAME}_intercom_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
list(APPEND ${PROJECT_NAME}_targets ${PROJECT_NAME}_intercom_node)


add_executable(${PROJECT_NAME}_server src/behavior_tree_server.cpp src/BehaviorTreeServer.cpp )
target_include_directories(${PROJECT_NAME}_server PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_server PRIVATE ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_server PRIVATE -Wall -Werror -Wfatal-errors -Wl,--no-undefined)
add_dependencies(${PROJECT_NAME}_server ${${PROJECT_NAME}_EXPORTED_TARGETS})
list(APPEND ${PROJECT_NAME}_targets ${PROJECT_NAME}_server)

if(ZMQ_FOUND)
    target_link_libraries(${PROJECT_NAME}_server PRIVATE
    zmq)
    target_link_libraries(${PROJECT_NAME}_node PRIVATE
    zmq)
    target_include_directories(${PROJECT_NAME}_server PRIVATE
        ${ZMQ_INCLUDE_DIRS} )
    target_compile_definitions(${PROJECT_NAME}_server PRIVATE ZMQ_FOUND)
endif()

install(TARGETS ${${PROJECT_NAME}_targets}
        ARCHIVE  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        INCLUDES DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Generate the XML files
add_custom_command(
    OUTPUT SO_TO_PALETTE
    COMMAND ${CMAKE_COMMAND} -Dbehavior_tree_cpp_DIR=${behavior_tree_cpp_DIR} -DCATKIN_DEVEL_PREFIX=${CATKIN_DEVEL_PREFIX} -P ${behavior_tree_cpp_DIR}/plugin_to_xml.cmake
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Running bt3_plugin_to_xml"
)

# Add a custom target that depends on the custom command
add_custom_target(generate_nodemodels ALL DEPENDS SO_TO_PALETTE)
add_dependencies(generate_nodemodels ${${PROJECT_NAME}_targets})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES palette_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY palettes/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/palettes)
