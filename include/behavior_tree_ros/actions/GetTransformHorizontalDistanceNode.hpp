#ifndef GET_TRANSFORM_HORIZONTAL_DISTANCE_HPP
#define GET_TRANSFORM_HORIZONTAL_DISTANCE_HPP

#include <tf/transform_datatypes.h>
#include <behaviortree_cpp_v3/action_node.h>

namespace BT_ROS
{
class GetTransformHorizontalDistanceNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~GetTransformHorizontalDistanceNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<tf::StampedTransform>("input", "TF transform"),
                     BT::OutputPort<double>("output", "2D distance from TF transform origin")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);
            const auto& input = getInput<tf::StampedTransform>("input");
            if(!input) { throw BT::RuntimeError { name() + ": " + input.error() }; }

            auto origin = input.value().getOrigin();

            origin.setZ(0.0);
            const auto& distance = origin.length();

            setOutput("output", distance);

            return BT::NodeStatus::SUCCESS;
        }
};
}

#endif
