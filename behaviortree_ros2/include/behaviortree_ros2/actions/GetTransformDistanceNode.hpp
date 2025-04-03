#ifndef GET_TRANSFORM_DISTANCE_HPP
#define GET_TRANSFORM_DISTANCE_HPP

//#include <tf/transform_datatypes.h>
#include "behaviortree_cpp/action_node.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace BT_ROS
{
class GetTransformDistanceNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~GetTransformDistanceNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<geometry_msgs::msg::TransformStamped>("input", "TF transform"),
                     BT::OutputPort<double>("output", "Distance form transform origin")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& input = getInput<geometry_msgs::msg::TransformStamped>("input");
            if(!input) { throw BT::RuntimeError { name() + ": " + input.error() }; }

            // Convert TransformStamped to tf2::Transform
            tf2::Transform transform;
            tf2::fromMsg(input.value().transform, transform);

            // Compute the distance
            double distance = transform.getOrigin().length();
            setOutput("output", distance);

            return BT::NodeStatus::SUCCESS;
        }
};
}

#endif
