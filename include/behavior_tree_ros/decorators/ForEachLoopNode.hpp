#ifndef FOR_EACH_LOOP_NODE_HPP
#define FOR_EACH_LOOP_NODE_HPP

#include <behaviortree_cpp/decorator_node.h>

#include "nlohmann/json.hpp"

namespace BT_ROS
{
template <typename T>
class ForEachLoopNode final : public BT::DecoratorNode
{
    public:
        using BT::DecoratorNode::DecoratorNode;
        ~ForEachLoopNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<T>("input", "Input sequence"),
                     BT::InputPort<BT::StringView>("message_field", "Field to fetch"),
                     BT::InputPort<bool>("break_on_child_failure", "Break loop on child failure?"),
                     BT::OutputPort<typename T::const_iterator::value_type>("output_element", "Output element variable"),
                     BT::OutputPort<size_t>("output_index", "Output index variable"),
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            if(!sequence_iterator_)
            { 
                const auto& message_field          = getInput<BT::StringView>("message_field");
                const auto& input_sequence         = getInput<T>("input_message");
                const auto& break_on_child_failure = getInput<bool>("break_on_child_failure");

                if(!message_field)          { throw BT::RuntimeError { "ForEachLoopNode: " + message_field.error() }; }
                if(!input_sequence)         { throw BT::RuntimeError { "ForEachLoopNode: " + input_sequence.error() }; }
                if(!break_on_child_failure) { throw BT::RuntimeError { "ForEachLoopNode: " + break_on_child_failure.error() }; }

                //TODO: do not copy
                input_sequence_         = input_sequence.value();
                break_on_child_failure_ = break_on_child_failure.value();
                sequence_iterator_      = std::make_unique<typename T::const_iterator>(input_sequence_.cbegin());
            }

            while(*sequence_iterator_ != input_sequence_.cend())
            {
                setOutput("output_index", std::distance(input_sequence_.cbegin(), *sequence_iterator_));
                setOutput("element", **sequence_iterator_);

                const auto child_status = child_node_->executeTick();

                if(child_status == BT::NodeStatus::FAILURE && break_on_child_failure_)
                {
                    sequence_iterator_.reset();
                    return BT::NodeStatus::FAILURE;
                }
                else if (child_status == BT::NodeStatus::RUNNING) { return child_status; }

                std::advance(*sequence_iterator_, 1);
            }

            sequence_iterator_.reset();
            return BT::NodeStatus::SUCCESS;
        }

        virtual void halt() override
        {
            sequence_iterator_.reset();
            BT::DecoratorNode::halt();
        }

    private:
        bool break_on_child_failure_ {};
        std::unique_ptr<typename T::const_iterator> sequence_iterator_ {};
        T input_sequence_;
};
}

#endif
