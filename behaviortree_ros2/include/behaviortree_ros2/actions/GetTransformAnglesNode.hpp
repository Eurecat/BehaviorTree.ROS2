#ifndef GET_TRANSFORM_ANGLES_HPP
#define GET_TRANSFORM_ANGLES_HPP

#include <angles/angles.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "behaviortree_cpp/action_node.h"

namespace BT_ROS
{
class GetTransformAnglesNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~GetTransformAnglesNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<geometry_msgs::msg::TransformStamped>("input", "TF transform"),
                     BT::OutputPort<double>("roll", "Roll angle [0,2PI] from TF transform origin"),
                     BT::OutputPort<double>("pitch", "Picth angle [0,2PI] from TF transform origin"),
                     BT::OutputPort<double>("yaw", "Yaw angle [0,2PI] from TF transform origin")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& input = getInput<geometry_msgs::msg::TransformStamped>("input");
            if(!input) { throw BT::RuntimeError { name() + ": " + input.error() }; }

            // Convert geometry_msgs::msg::Transform to tf2::Transform
            tf2::Transform transform;
            tf2::fromMsg(input.value().transform, transform);

            // Extract rotation as a tf2::Quaternion
            tf2::Quaternion q = transform.getRotation();
            tf2::Matrix3x3 m(q);

            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // Normalize angles
            roll = angles::normalize_angle_positive(roll);
            pitch = angles::normalize_angle_positive(pitch);
            yaw = angles::normalize_angle_positive(yaw);

            // printf("-------------------- \n");
            // printf("Roll: %.2f\n", roll);
            // printf("Pitch: %.2f\n", pitch);
            // printf("Yaw: %.2f\n", yaw);
            // printf("-------------------- \n");

            setOutput("roll", roll);
            setOutput("pitch", pitch);
            setOutput("yaw", yaw);

            return BT::NodeStatus::SUCCESS;
        }
};
}

#endif
