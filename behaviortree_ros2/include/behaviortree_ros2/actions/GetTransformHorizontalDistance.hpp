#ifndef GET_TRANSFORM_HORIZONTAL_DISTANCE_HPP
#define GET_TRANSFORM_HORIZONTAL_DISTANCE_HPP

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "behaviortree_cpp/action_node.h"

namespace BT_ROS
{
class GetTransformHorizontalDistanceNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~GetTransformHorizontalDistanceNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<geometry_msgs::msg::TransformStamped>("input", "TF transform"),
                     BT::OutputPort<double>("output", "2D distance from TF transform origin")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& input = getInput<geometry_msgs::msg::TransformStamped>("input");
            if(!input) { throw BT::RuntimeError { name() + ": " + input.error() }; }

            // Convert geometry_msgs::msg::Transform to tf2::Transform
            tf2::Transform transform;
            tf2::fromMsg(input.value().transform, transform);

            auto origin = transform.getOrigin();

            origin.setZ(0.0);
            const auto& distance = origin.length();

            setOutput("output", distance);

            return BT::NodeStatus::SUCCESS;
        }
};
}

#endif