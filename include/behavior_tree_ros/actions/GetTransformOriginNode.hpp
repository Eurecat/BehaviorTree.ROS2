#ifndef GET_TRANSFORM_ORIGIN_HPP
#define GET_TRANSFORM_ORIGIN_HPP

#include <tf/transform_datatypes.h>
#include <behaviortree_cpp/action_node.h>

namespace BT_ROS
{
class GetTransformOriginNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~GetTransformOriginNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<tf::StampedTransform>("input", "TF transform"),
                     BT::OutputPort<double>("x", "Origin x coordinate"),
                     BT::OutputPort<double>("y", "Origin y coordinate"),
                     BT::OutputPort<double>("z", "Origin z coordinate")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);

            const auto& input = getInput<tf::StampedTransform>("input");
            if(!input) { throw BT::RuntimeError { name() + ": " + input.error() }; }

            const auto& origin = input.value().getOrigin();

            setOutput("x", origin.getX());
            setOutput("y", origin.getY());
            setOutput("z", origin.getZ());

            return BT::NodeStatus::SUCCESS;
        }
};
}

#endif
