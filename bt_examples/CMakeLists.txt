cmake_minimum_required(VERSION 3.8)
project(bt_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)

include_directories( include/plugins/action  )

set(THIS_PACKAGE_DEPS
    behaviortree_ros2
    std_msgs
    std_srvs
    btcpp_ros2_interfaces
)

add_library(bt_plugins SHARED
  src/plugins/action/sleep.cpp
  src/plugins/register_plugins.cpp
)


ament_target_dependencies(bt_plugins ${THIS_PACKAGE_DEPS})
target_compile_definitions(bt_plugins PRIVATE BT_PLUGIN_EXPORT)



# Install the shared library into the desired directory.
install(TARGETS bt_plugins
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

ament_export_libraries(bt_plugins)


add_executable(bt_executor src/bt_executor.cpp)
ament_target_dependencies(bt_executor  ${THIS_PACKAGE_DEPS})

install(TARGETS
  bt_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  behavior_trees
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

#ament_export_dependencies(behaviortree_ros2 btcpp_ros2_interfaces)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
