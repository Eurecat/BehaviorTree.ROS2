cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(behavior_tree_ros VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(behavior_tree_core 1.0 REQUIRED)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        ros_type_introspection
        topic_tools
        message_generation
)

add_service_files(
    FILES
        GetLoadedPlugins.srv
)

generate_messages(DEPENDENCIES)

catkin_package(
    CATKIN_DEPENDS 
        roscpp
        ros_type_introspection
        topic_tools
        message_runtime
    INCLUDE_DIRS
        include
    DEPENDS
        behavior_tree_core
    LIBRARIES
        ${PROJECT_NAME}
)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${CATKIN_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} INTERFACE ${behavior_tree_core_LIBRARIES})
target_compile_options(${PROJECT_NAME} INTERFACE -Wl,--no-undefined)
list(APPEND ${PROJECT_NAME}_targets ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_node src/behavior_tree_node.cpp src/BehaviorTreeNode.cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${catkin_LIBRARIES} ${behavior_tree_core_LIBRARIES})
target_include_directories(${PROJECT_NAME}_node PRIVATE ${catkin_INCLUDE_DIRS} include)
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Werror -Wfatal-errors -Wl,--no-undefined)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
list(APPEND ${PROJECT_NAME}_targets ${PROJECT_NAME}_node)

install(TARGETS ${${PROJECT_NAME}_targets}
        ARCHIVE  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        INCLUDES DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
