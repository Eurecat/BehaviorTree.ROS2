#ifndef GET_SIZE_NODE_HPP
#define GET_SIZE_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>

namespace BT_ROS
{
template <class T>
class GetSizeNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~GetSizeNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<T>("input", "Input sequence"),
                     BT::OutputPort<size_t>("output", "Sequence size output") };
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);

            const auto& input = getInput<T>("input");
            if(!input) { throw BT::RuntimeError { name() + ": " + input.error() }; }

            setOutput("output", input.value().size());

            return BT::NodeStatus::SUCCESS;
        }
};
}

#endif
