#ifndef COPY_NODE_HPP
#define COPY_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>

namespace BT_ROS
{
template <class T>
class CopyNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~CopyNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<T>("input", "Input variable"),
                     BT::OutputPort<T>("output", "Output variable to copy to") };
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);

            const auto& input = getInput<T>("input");
            if(!input) { throw BT::RuntimeError { name() + ": " + input.error() }; }

            setOutput("output", input.value());

            return BT::NodeStatus::SUCCESS;
        }
};
}

#endif
