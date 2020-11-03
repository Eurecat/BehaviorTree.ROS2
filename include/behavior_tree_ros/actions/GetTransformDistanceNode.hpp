#ifndef GET_TRANSFORM_DISTANCE_HPP
#define GET_TRANSFORM_DISTANCE_HPP

#include <tf/transform_datatypes.h>
#include <behaviortree_cpp/action_node.h>

namespace BT_ROS
{
class GetTransformDistanceNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~GetTransformDistanceNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<tf::StampedTransform>("input", "TF transform"),
                     BT::OutputPort<double>("output", "Distance form transform origin")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);
            const auto& input = getInput<tf::StampedTransform>("input");
            if(!input) { throw BT::RuntimeError { name() + ": " + input.error() }; }

            const auto& distance = input.value().getOrigin().length();
            setOutput("output", distance);

            return BT::NodeStatus::SUCCESS;
        }
};
}

#endif
