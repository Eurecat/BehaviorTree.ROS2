#ifndef FOR_EACH_LOOP_NODE_HPP
#define FOR_EACH_LOOP_NODE_HPP

#include <behaviortree_cpp/decorator_node.h>

#include "nlohmann/json.hpp"

namespace BT_ROS
{
template <class T>
class ForEachLoopNode final : public BT::DecoratorNode
{
    public:
        using BT::DecoratorNode::DecoratorNode;
        ~ForEachLoopNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<T>("input", "Input sequence"),
                     BT::InputPort<bool>("break_on_child_failure", "Break loop on child failure?"),
                     BT::OutputPort<typename T::const_iterator::value_type>("output_element", "Output element variable"),
                     BT::OutputPort<size_t>("output_index", "Output index variable"),
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);
            if(!current_iterator_) { init(); }

            while(*current_iterator_ != *end_iterator_)
            {
                setOutput("output_index", std::distance(*begin_iterator_, *current_iterator_));
                setOutput("element", **current_iterator_);

                const auto child_status = child_node_->executeTick();

                if(child_status == BT::NodeStatus::FAILURE && break_on_child_failure_)
                {
                    reset();
                    return BT::NodeStatus::FAILURE;
                }
                else if (child_status == BT::NodeStatus::RUNNING) { return child_status; }

                std::advance(*current_iterator_, 1);
            }

            reset();
            return BT::NodeStatus::SUCCESS;
        }

        virtual void halt() override
        {
            reset();
            BT::DecoratorNode::halt();
        }

    private:
        void reset()
        {
            current_iterator_.reset();
            begin_iterator_.reset();
            end_iterator_.reset();
        }

        void init()
        {
            const auto& input_sequence         = getInput<T>("input");
            const auto& break_on_child_failure = getInput<bool>("break_on_child_failure");

            if(!input_sequence)         { throw BT::RuntimeError { name() + ": " + input_sequence.error() }; }
            if(!break_on_child_failure) { throw BT::RuntimeError { name() + ": " + break_on_child_failure.error() }; }

            input_sequence_ = input_sequence.value();
            break_on_child_failure_ = break_on_child_failure.value();

            current_iterator_ = std::make_unique<typename T::const_iterator>(input_sequence_.cbegin());
            begin_iterator_   = std::make_unique<typename T::const_iterator>(input_sequence_.cbegin());
            end_iterator_     = std::make_unique<typename T::const_iterator>(input_sequence_.cend());
        }

    private:
        T input_sequence_ {};
        bool break_on_child_failure_ {};

        //TODO: iterators may be invalidated if the blackboard entry is modified. Think of a way to
        //avoid this, or at least check on runtime if they are still valid
        std::unique_ptr<typename T::const_iterator> current_iterator_ {};
        std::unique_ptr<typename T::const_iterator> begin_iterator_   {};
        std::unique_ptr<typename T::const_iterator> end_iterator_     {};
};
}

#endif
