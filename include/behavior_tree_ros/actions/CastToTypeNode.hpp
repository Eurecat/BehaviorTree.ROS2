#ifndef CAST_TO_TYPE_NODE_HPP
#define CAST_TO_TYPE_NODE_HPP

#include <behaviortree_cpp/basic_types.h>

namespace BT_ROS
{
template <typename T>
class CastToTypeNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~CastToTypeNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<BT::Any>("input", "Type-deleted input variable"),
                     BT::OutputPort<T>("output", "Casted output value")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& input = getInput<BT::Any>("input");

            if(!input) { throw BT::RuntimeError { "CastToTypeNode: " + input.error() }; }
            if(input.value().type() != typeid(T)) { return BT::NodeStatus::FAILURE; }

            setOutput("output", input.value().template cast<T>());

            return BT::NodeStatus::SUCCESS;
        }
};
}

#endif
