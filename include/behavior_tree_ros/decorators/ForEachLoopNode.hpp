#ifndef FOR_EACH_LOOP_NODE_HPP
#define FOR_EACH_LOOP_NODE_HPP

#include <behaviortree_cpp/decorator_node.h>

#include "nlohmann/json.hpp"

namespace BT_ROS
{
class ForEachLoopNode final : public BT::DecoratorNode
{
    public:
        using BT::DecoratorNode::DecoratorNode;
        ~ForEachLoopNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<nlohmann::json>("input", "Serialized ROS message"),
                     BT::InputPort<std::string>("message_field", "Field to fetch"),
                     BT::OutputPort<std::string>("output_element", "Output element variable"),
                     BT::OutputPort<std::string>("output_index", "Output index variable"),
                     BT::InputPort<bool>("break_on_child_failure", "Break loop on child failure?"),
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            try
            {
                const auto& output_index   = getInput<std::string>("output_index");
                const auto& output_element = getInput<std::string>("output_element");

                if(!sequence_iterator_)
                { 
                    const auto& message_field = getInput<std::string>("message_field");
                    nlohmann::json::json_pointer pointer(message_field.value());
                    input_sequence_ = getInput<nlohmann::json>("input_message").value().at(pointer);

                    sequence_iterator_ = input_sequence_.cbegin();
                }

                while(sequence_iterator_ != input_sequence_.cend())
                {
                    if(output_index)
                    {
                        setOutput(output_index.value(), std::distance(input_sequence_.cbegin(), sequence_iterator_.value()));
                    }

                    if(output_element)
                    {
                        setOutput(output_element.value(), *sequence_iterator_.value());
                    }

                    const auto child_status = child_node_->executeTick();

                    if(child_status == BT::NodeStatus::FAILURE && break_on_child_failure_)
                    {
                        //sequence_iterator_.reset();
                        return BT::NodeStatus::FAILURE;
                    }
                    else if (child_status == BT::NodeStatus::RUNNING) { return child_status; }

                    std::advance(sequence_iterator_.value(), 1);
                }

                //sequence_iterator_.reset();
                return BT::NodeStatus::SUCCESS;
            }
            catch(const std::runtime_error&)        { return BT::NodeStatus::FAILURE; }
            //catch(const BT::bad_optional_access&)   { return BT::NodeStatus::FAILURE; }
            catch(const nlohmann::json::exception&) { return BT::NodeStatus::FAILURE; }
        }

        /*
        virtual void onInit() override
        {
            if(!getInput("break_on_child_failure", break_on_child_failure_))
            {
                throw std::runtime_error {"ForEachLoopNode: missing or incorrect break_on_child_failure parameter"};
            }
        }
        */

        virtual void halt() override
        {
            //sequence_iterator_.reset();
            BT::DecoratorNode::halt();
        }

    private:
        bool break_on_child_failure_ {};
        BT::Optional<nlohmann::json::const_iterator> sequence_iterator_ {};
        nlohmann::json input_sequence_;
};
}

#endif
