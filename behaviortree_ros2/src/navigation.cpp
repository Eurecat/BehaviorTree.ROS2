#include "behaviortree_ros2/behaviortree_ros2.hpp"

//#include "behaviortree_ros2/actions/ros_log.hpp"
//#include "behaviortree_ros2/actions/GetTransformAnglesNode.hpp"
//#include "behaviortree_ros2/actions/GetTransformDistanceNode.hpp"
//#include "behaviortree_ros2/actions/GetTransformHorizontalDistance.hpp"
//#include "behaviortree_ros2/actions/LookupTransformNode.hpp"

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
   
//TEST
#include <std_msgs/msg/int8_multi_array.hpp>
#include "btcpp_ros2_interfaces/action/sleep.hpp"
#include "btcpp_ros2_interfaces/action/execute_tree.hpp"
#include "btcpp_ros2_interfaces/msg/custom_msg.hpp"



// Custom ROS2-related nodes creation (todo: put into a separate library)
class CheckPoseReached : public BT::StatefulActionNode
{
public:
  CheckPoseReached(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      // Target pose
      BT::InputPort<double>("d_x"),
      BT::InputPort<double>("d_y"),
      BT::InputPort<double>("d_z"),
      BT::InputPort<double>("d_qx"),
      BT::InputPort<double>("d_qy"),
      BT::InputPort<double>("d_qz"),
      BT::InputPort<double>("d_qw"),
      // Current pose
      BT::InputPort<double>("x"),
      BT::InputPort<double>("y"),
      BT::InputPort<double>("z"),
      BT::InputPort<double>("qx"),
      BT::InputPort<double>("qy"),
      BT::InputPort<double>("qz"),
      BT::InputPort<double>("qw"),
      // Thresholds
      BT::InputPort<double>("pos_tolerance", 0.05, "Position tolerance in meters"),
      BT::InputPort<double>("ang_tolerance", 0.1, "Orientation tolerance in radians")
    };
  }

  BT::NodeStatus onStart() override
  {
    // Read all inputs
    if (!getPoseInputs(target_, "d_x", "d_y", "d_z", "d_qx", "d_qy", "d_qz", "d_qw")) {
      std::cerr << "[CheckPoseReached] Missing target pose input!" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    if (!getPoseInputs(current_, "x", "y", "z",
                       "qx", "qy", "qz", "qw")) {
      std::cerr << "[CheckPoseReached] Missing current pose input!" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    getInput("pos_tolerance", pos_tol_);
    getInput("ang_tolerance", ang_tol_);

    std::cout << "[CheckPoseReached] Checking if pose is reached..." << std::endl;
    return BT::NodeStatus::RUNNING;
  }




  BT::NodeStatus onRunning() override
  {
    // Compute position difference
    double dx = target_.x - current_.x;
    double dy = target_.y - current_.y;
    double dz = target_.z - current_.z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    // Compute orientation difference (using quaternion dot product)
    double dot = target_.qx * current_.qx + target_.qy * current_.qy +
                 target_.qz * current_.qz + target_.qw * current_.qw;
    double angle_diff = 2 * std::acos(std::abs(dot));

    if (dist < pos_tol_ && angle_diff < ang_tol_)
    {
      std::cout << "[CheckPoseReached] Target reached! dist=" << dist
                << ", angle=" << angle_diff << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      std::cout << "[CheckPoseReached] Not yet reached. dist=" << dist
                << ", angle=" << angle_diff << std::endl;
      return BT::NodeStatus::FAILURE;
    }
  }

  void onHalted() override
  {
    std::cout << "[CheckPoseReached] Halted." << std::endl;
  }

private:
  struct Pose {
    double x, y, z;
    double qx, qy, qz, qw;
  } target_, current_;

  double pos_tol_ = 0.05;
  double ang_tol_ = 0.1;

  bool getPoseInputs(Pose& pose,
                     const std::string& x_key, const std::string& y_key, const std::string& z_key,
                     const std::string& qx_key, const std::string& qy_key,
                     const std::string& qz_key, const std::string& qw_key)
  {
    return getInput(x_key, pose.x) &&
           getInput(y_key, pose.y) &&
           getInput(z_key, pose.z) &&
           getInput(qx_key, pose.qx) &&
           getInput(qy_key, pose.qy) &&
           getInput(qz_key, pose.qz) &&
           getInput(qw_key, pose.qw);
  }
};





class SetPoseGoal : public BT::SyncActionNode
{
public:
  SetPoseGoal(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      // Input pose parameters
      BT::InputPort<double>("x"),
      BT::InputPort<double>("y"),
      BT::InputPort<double>("z"),
      BT::InputPort<double>("qx"),
      BT::InputPort<double>("qy"),
      BT::InputPort<double>("qz"),
      BT::InputPort<double>("qw"),

      // Output ports to blackboard
      BT::OutputPort<double>("d_x"),
      BT::OutputPort<double>("d_y"),
      BT::OutputPort<double>("d_z"),
      BT::OutputPort<double>("d_qx"),
      BT::OutputPort<double>("d_qy"),
      BT::OutputPort<double>("d_qz"),
      BT::OutputPort<double>("d_qw")
    };
  }

  BT::NodeStatus tick() override
  {
    double x, y, z, qx, qy, qz, qw;

    if (!getInput("x", x) || !getInput("y", y) || !getInput("z", z) ||
        !getInput("qx", qx) || !getInput("qy", qy) ||
        !getInput("qz", qz) || !getInput("qw", qw))
    {
      std::cerr << "[SetPoseGoal] Missing one or more input values!" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    // Set outputs on the blackboard
    setOutput("d_x", x);
    setOutput("d_y", y);
    setOutput("d_z", z);
    setOutput("d_qx", qx);
    setOutput("d_qy", qy);
    setOutput("d_qz", qz);
    setOutput("d_qw", qw);

    return BT::NodeStatus::SUCCESS;
  }
};

using namespace BT;

/*
BT_REGISTER_ROS_NODES(factory, params)
{
  
    factory.registerNodeType<SetPoseGoal>("SetPoseGoal");
    factory.registerNodeType<CheckPoseReached>("CheckPoseReached");

};*/