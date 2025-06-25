#include <algorithm>  
#include <gtest/gtest.h>  
#include <rclcpp/rclcpp.hpp>  
#include "behaviortree_ros2/tools/client/tree_execution_client.hpp"


TEST(package_name, BTROS2_TEST) {

  auto node = std::make_shared<TreeExecutionClient>("ExampleTree1");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  ASSERT_EQ("test", "value");
}



int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);    
    rclcpp::init(argc, argv);
    return RUN_ALL_TESTS();
  }
