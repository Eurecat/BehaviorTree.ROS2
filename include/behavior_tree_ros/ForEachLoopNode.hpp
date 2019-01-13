#ifndef FOR_EACH_LOOP_NODE_HPP
#define FOR_EACH_LOOP_NODE_HPP

#include <behaviortree_cpp/decorator_node.h>

#include "nlohmann/json.hpp"

namespace BT_ROS
{
class ForEachLoopNode final : public BT::DecoratorNode
{
    public:
        ForEachLoopNode(const std::string& _name, const BT::NodeParameters& _params) : DecoratorNode(_name, _params)
        {}
        ~ForEachLoopNode() = default;

        static const BT::NodeParameters& requiredNodeParameters()
        {
            static BT::NodeParameters params { { "input_message", "" },  {"message_field", ""},
                                               { "output_element", "" }, {"output_index", "" },
                                               { "break_on_child_failure", "true" } };
            return params;
        }

        virtual BT::NodeStatus tick() override
        {
            try
            {
                const auto& output_index   = getParam<std::string>("output_index");
                const auto& output_element = getParam<std::string>("output_element");

                if(!sequence_iterator_)
                { 
                    const auto& message_field = getParam<std::string>("message_field");
                    nlohmann::json::json_pointer pointer(message_field.value());
                    input_sequence_ = getParam<nlohmann::json>("input_message").value().at(pointer);

                    sequence_iterator_ = input_sequence_.cbegin();
                }

                if(sequence_iterator_.value() == input_sequence_.cend())
                {
                    sequence_iterator_.reset();
                    return BT::NodeStatus::SUCCESS;
                }

                if(output_index)
                {
                    blackboard()->set(output_index.value(), std::distance(input_sequence_.cbegin(), sequence_iterator_.value()));
                }

                if(output_element)
                {
                    blackboard()->set(output_element.value(), *sequence_iterator_.value());
                }

                const auto child_status = child_node_->executeTick();

                if(child_status == BT::NodeStatus::FAILURE && break_on_child_failure_)
                {
                    sequence_iterator_.reset();
                    return BT::NodeStatus::FAILURE;
                }

                std::advance(sequence_iterator_.value(), 1);

                return BT::NodeStatus::RUNNING;
            }
            catch(const std::runtime_error&)        { return BT::NodeStatus::FAILURE; }
            catch(const BT::bad_optional_access&)   { return BT::NodeStatus::FAILURE; }
            catch(const nlohmann::json::exception&) { return BT::NodeStatus::FAILURE; }
        }

        virtual void onInit() override
        {
            if(!getParam("break_on_child_failure", break_on_child_failure_))
            {
                throw std::runtime_error {"ForEachLoopNode: missing or incorrect break_on_child_failure parameter"};
            }
        }

        virtual void halt() override {}

    private:
        bool break_on_child_failure_ {};
        BT::optional<nlohmann::json::const_iterator> sequence_iterator_ {};
        nlohmann::json input_sequence_;
};
}

#endif
