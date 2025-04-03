cmake_minimum_required(VERSION 3.16)
project(behaviortree_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options(-fPIC)

set(THIS_PACKAGE_DEPS
    rclcpp
    rclcpp_action
    ament_index_cpp
    behaviortree_cpp
    behaviortree_eut_plugins
    btcpp_ros2_interfaces
    generate_parameter_library
    rosx_introspection
    std_msgs
    std_srvs
    geometry_msgs
    angles
    tf2
    tf2_geometry_msgs
    tf2_ros
    )

find_package(ament_cmake REQUIRED)

foreach(PACKAGE ${THIS_PACKAGE_DEPS})
    find_package(${PACKAGE} REQUIRED )
endforeach()

generate_parameter_library(
    bt_executor_parameters
    src/bt_executor_parameters.yaml)

add_library(${PROJECT_NAME}
    src/bt_ros2.cpp
    src/bt_utils.cpp
    src/tree_execution_server.cpp )

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPS})

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}  bt_executor_parameters)

# Executable to generate xml from .so plugin for "simple" and ros2 based bt plugins .so
add_executable(bt_ros_plugins_to_xml         src/tools/bt_ros_plugins_to_xml.cpp )
include_directories(${behaviortree_cpp_INCLUDE_DIRS} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(bt_ros_plugins_to_xml behaviortree_cpp rclcpp)

# Build btros2 plugin library
add_library(${PROJECT_NAME}_nodemodels SHARED src/plugin.cpp)
target_include_directories(${PROJECT_NAME}_nodemodels PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_definitions(${PROJECT_NAME}_nodemodels PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(${PROJECT_NAME}_nodemodels ${THIS_PACKAGE_DEPS})


######################################################
# INSTALL

install(TARGETS bt_ros_plugins_to_xml
        DESTINATION lib/${PROJECT_NAME} )

install(DIRECTORY include/  DESTINATION include/)

ament_export_targets(${PROJECT_NAME}Targets  HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPS})

install(
  TARGETS ${PROJECT_NAME} bt_executor_parameters
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# Recover subfolder path for plugins
execute_process(
    COMMAND bash -c "xmllint --xpath 'string(//description/plugins/@folder_path)' plugins_description.xml | awk '{print \$1\}'"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE PLUGINS_FOLDER_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

install(TARGETS
  ${PROJECT_NAME}_nodemodels
  LIBRARY DESTINATION share/${PROJECT_NAME}/${PLUGINS_FOLDER_PATH}
  ARCHIVE DESTINATION share/${PROJECT_NAME}/${PLUGINS_FOLDER_PATH}
  RUNTIME DESTINATION share/${PROJECT_NAME}/${PLUGINS_FOLDER_PATH}
  )

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/libbt_${PROJECT_NAME}_nodemodels.xml
  COMMAND $<TARGET_FILE:bt_ros_plugins_to_xml> 
          $<TARGET_FILE:${PROJECT_NAME}_nodemodels>
          ${CMAKE_CURRENT_BINARY_DIR}/libbt_${PROJECT_NAME}_nodemodels.xml
  DEPENDS bt_ros_plugins_to_xml ${PROJECT_NAME}_nodemodels
  COMMENT "Generating XML file from plugin .so"
)

add_custom_target(generate_xml ALL
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/libbt_${PROJECT_NAME}_nodemodels.xml
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libbt_${PROJECT_NAME}_nodemodels.xml
        DESTINATION share/${PROJECT_NAME}/${PLUGINS_FOLDER_PATH})

# Register the plugin description file as a resource
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/plugins_description.xml
               ${CMAKE_CURRENT_BINARY_DIR}/plugins_description.xml COPYONLY)

ament_index_register_resource(
  "behavior_tree_plugin"
  CONTENT_FILE "${CMAKE_CURRENT_SOURCE_DIR}/plugins_description.xml"
)

# Register the palette description file as a resource
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/palettes_description.xml
               ${CMAKE_CURRENT_BINARY_DIR}/palettes_description.xml COPYONLY)

ament_index_register_resource(
  "behavior_tree_palette"
  CONTENT_FILE "${CMAKE_CURRENT_SOURCE_DIR}/palettes_description.xml"
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
