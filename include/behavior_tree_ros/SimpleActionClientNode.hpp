#ifndef SIMPLE_ACTION_CLIENT_NODE_HPP
#define SIMPLE_ACTION_CLIENT_NODE_HPP

#include <mutex>
#include <atomic>
#include <memory>

#include <behaviortree_cpp/action_node.h>
#include <actionlib/client/simple_action_client.h>

#include "behavior_tree_ros/policies/serialization_policies.hpp"
#include "behavior_tree_ros/policies/deserialization_policies.hpp"

namespace BT_ROS
{
template <class ActionType,  template <class> class GoalDeserializationPolicy,
                             template <class> class ResultSerializationPolicy,
                             template <class> class FeedbackSerializationPolicy>
class SimpleActionClientNode final : public BT::ActionNodeBase,
                                     public GoalDeserializationPolicy<typename ActionType::_action_goal_type>,
                                     public ResultSerializationPolicy<typename ActionType::_action_result_type>,
                                     public FeedbackSerializationPolicy<typename ActionType::_action_feedback_type>
{
    private:
        using SimpleClient    = actionlib::SimpleActionClient<ActionType>;
        using SimpleClientPtr = std::unique_ptr<SimpleClient>;

        using GoalState = actionlib::SimpleClientGoalState;

        using GoalPolicy     = GoalDeserializationPolicy<typename ActionType::_action_goal_type>;
        using ResultPolicy   = ResultSerializationPolicy<typename ActionType::_action_result_type>;
        using FeedbackPolicy = FeedbackSerializationPolicy<typename ActionType::_action_feedback_type>;

    public:
        SimpleActionClientNode(const std::string& _name, const BT::NodeConfiguration& _config) : ActionNodeBase(_name, _config)
        {
            const auto& action = getInput<std::string>("action");
            if(!action) { throw BT::RuntimeError { name() + ": " + action.error() }; }

            client_ = std::make_unique<SimpleClient>(node_handle_, action.value(), false, {}, {},
                                                     boost::bind(&SimpleActionClientNode::FeedbackCallback, this, _1));

            client_->waitForServer();
        }

        ~SimpleActionClientNode() = default;

        static BT::PortsList providedPorts()
        {
            BT:: PortsList ports { BT::InputPort<std::string>("action", "Actionlib action server name"),
                                   BT::OutputPort<GoalState>("state", "Actionlib reported state")
                                 };

            const auto& goal_ports = GoalPolicy::requiredPorts();
            ports.insert(goal_ports.cbegin(), goal_ports.cend());

            const auto& feedback_ports = FeedbackPolicy::requiredPorts("serialized_feedback");
            ports.insert(feedback_ports.cbegin(), feedback_ports.cend());

            const auto& result_ports = ResultPolicy::requiredPorts("serialized_result");
            ports.insert(result_ports.cbegin(), result_ports.cend());

            return ports;
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);

            if(!goal_sent_)
            {
                const auto& goal_msg = goal_policy_.buildMessage(*this);
                client_->sendGoal(goal_msg);

                goal_sent_ = true;
            }

            // Get state, save it in the output and save it in the output variable
            // TODO: make the output variable optional
            const auto& goal_state = client_->getState();
            setOutput("state", goal_state);

            // TODO: is this the best way to handle feedback msgs
            {
                std::unique_lock<std::mutex> lock (feedback_mutex_);
                if(new_feedback_)
                {
                    new_feedback_ = false;
                    response_policy_.onNewMessage(new_feedback_, *this);
                }
            }

            // TODO: is this pointer null?
            const auto& result_ptr = client_->getResult();
            result_policy_.onNewMessage(*result_ptr, *this);

            const BT::NodeStatus status = GoalState2Status(goal_state);

            // TODO: should we wait a bit more before checking this?
            if(status != BT::NodeStatus::RUNNING) { goal_sent_ = false; }

            return status;
        }

        virtual void halt() override
        {
            if(client_) { client_->cancelGoal(); }
        }
    
    private:
        // TODO: move this out of here
        BT::NodeStatus GoalState2Status(const GoalState& _state)
        {
            return BT::NodeStatus::SUCCESS;

            switch(_state)
            {
                case GoalState::StateEnum::PENDING:
                    return BT::NodeStatus::IDLE;
                    break;
                case GoalState::StateEnum::ACTIVE:
                case GoalState::StateEnum::RECALLING:
                case GoalState::StateEnum::PREEMPTING:
                    return BT::NodeStatus::RUNNING;
                    break;
                case GoalState::StateEnum::PREEMPTED:
                case GoalState::StateEnum::ABORTED:
                case GoalState::StateEnum::REJECTED:
                case GoalState::StateEnum::LOST:
                    return BT::NodeStatus::FAILURE;
                    break;
                case GoalState::StateEnum::SUCCEEDED:
                    return BT::NodeStatus::SUCCESS;
                    break;
                default:
                    return BT::NodeStatus::FAILURE;
            }
        }

        void FeedbackCallback(const FeedbackConstPtr& _feedback_msg)
        {
            std::unique_lock<std::mutex> lock (feedback_mutex_);

            new_feedback_ = true;
            feedback_msg_ = *_feedback_msg;
        }

    private:
        ros::NodeHandle node_handle_;
        SimpleClientPtr client_;

        GoalPolicy     goal_policy_     {};
        ResultPolicy   result_policy_   {};
        FeedbackPolicy feedback_policy_ {};

        bool goal_sent_ { false };

        std::mutex        feedback_mutex_;
        std::atomic<bool> new_feedback_;
        Feedback          feedback_msg_;
};

//Shortcut alias
template <class ActionType>
using SimpleActionClient = SimpleActionClientNode<ActionType, NoDeserialization,
                                                              NoSerialization,
                                                              NoSerialization>;

template <class ActionType>
using AutomaticSimpleActionClient = SimpleActionClientNode<ActionType, AutomaticDeserialization,
                                                                       JsonSerialization,
                                                                       JsonSerialization>;
}

#endif
