#ifndef GET_TRANSFORM_ANGLES_HPP
#define GET_TRANSFORM_ANGLES_HPP

#include <angles/angles.h>
#include <tf/transform_datatypes.h>
#include <behaviortree_cpp_v3/action_node.h>

namespace BT_ROS
{
class GetTransformAnglesNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~GetTransformAnglesNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<tf::StampedTransform>("input", "TF transform"),
                     BT::OutputPort<double>("roll", "Roll angle [0,2PI] from TF transform origin"),
                     BT::OutputPort<double>("pitch", "Picth angle [0,2PI] from TF transform origin"),
                     BT::OutputPort<double>("yaw", "Yaw angle [0,2PI] from TF transform origin")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& input = getInput<tf::StampedTransform>("input");
            if(!input) { throw BT::RuntimeError { name() + ": " + input.error() }; }

            tf::Quaternion q = input.value().getRotation();
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
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
