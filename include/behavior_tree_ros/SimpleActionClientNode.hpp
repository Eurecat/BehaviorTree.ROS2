#ifndef SIMPLE_ACTION_CLIENT_NODE_HPP
#define SIMPLE_ACTION_CLIENT_NODE_HPP

#include <mutex>
#include <atomic>
#include <memory>

#include <behaviortree_cpp_v3/action_node.h>
#include <actionlib/client/simple_action_client.h>

#include "behavior_tree_ros/policies/serialization_policies.hpp"
#include "behavior_tree_ros/policies/deserialization_policies.hpp"

namespace BT
{
    template <>
    std::string toStr<actionlib::SimpleClientGoalState>(actionlib::SimpleClientGoalState goalState)
    {
        return goalState.toString() + ": " + goalState.getText();
    }
}

namespace BT_ROS
{

template <class ActionType,  template <class> class GoalDeserializationPolicy,
                             template <class> class ResultSerializationPolicy,
                             template <class> class FeedbackSerializationPolicy>
class SimpleActionClientNode final : public BT::CoroActionNode,
                                     public GoalDeserializationPolicy<typename ActionType::_action_goal_type::_goal_type>,
                                     public ResultSerializationPolicy<typename ActionType::_action_result_type::_result_type>,
                                     public FeedbackSerializationPolicy<typename ActionType::_action_feedback_type::_feedback_type>
{
    private:
        using SimpleClient    = actionlib::SimpleActionClient<ActionType>;
        using SimpleClientPtr = std::unique_ptr<SimpleClient>;

        using Goal     = typename ActionType::_action_goal_type::_goal_type;
        using Result   = typename ActionType::_action_result_type::_result_type;
        using Feedback = typename ActionType::_action_feedback_type::_feedback_type;

        using GoalState = actionlib::SimpleClientGoalState;

        using GoalPolicy     = GoalDeserializationPolicy<Goal>;
        using ResultPolicy   = ResultSerializationPolicy<Result>;
        using FeedbackPolicy = FeedbackSerializationPolicy<Feedback>;

    public:
        SimpleActionClientNode(const std::string& _name, const BT::NodeConfiguration& _config) : CoroActionNode(_name, _config)
        {
            const auto& action = getInput<std::string>("action");
            if(!action) { throw BT::RuntimeError { name() + ": " + action.error() }; }

            client_ = std::make_unique<SimpleClient>(node_handle_, action.value(), false);
        }

        ~SimpleActionClientNode(){ halt();}

        static BT::PortsList providedPorts()
        {
            BT:: PortsList ports { BT::InputPort<std::string>("action", "Actionlib action server name"),
                                   BT::OutputPort<GoalState>("state", "Actionlib reported state")
                                 };

            const auto& goal_ports = GoalPolicy::requiredPorts();
            ports.insert(goal_ports.cbegin(), goal_ports.cend());

            const auto& feedback_ports = FeedbackPolicy::requiredPorts("feedback");
            ports.insert(feedback_ports.cbegin(), feedback_ports.cend());

            const auto& result_ports = ResultPolicy::requiredPorts("result");
            ports.insert(result_ports.cbegin(), result_ports.cend());

            return ports;
        }

        virtual BT::NodeStatus tick() override
        {
            if (client_->isServerConnected())
            {
                {
                    const auto& goal_msg = goal_policy_.buildMessage(*this);
                    client_->sendGoal(goal_msg, {}, {}, boost::bind(&SimpleActionClientNode::FeedbackCallback, this, _1));
                    goal_finished_ = false;
                }

                while(!goal_finished_)
                {
                    // Check connection to prevent lock if server dies processing goal
                    if (!client_->isServerConnected()) { return BT::NodeStatus::FAILURE; }

                    // Get state, save it in the output and save it in the output variable
                    goal_state_ = client_->getState();
                    setOutput("state", goal_state_);

                    {
                        std::unique_lock<std::mutex> lock (feedback_mutex_);
                        if(new_feedback_)
                        {
                            new_feedback_ = false;
                            feedback_policy_.onNewMessage(feedback_msg_, *this, "feedback");
                        }
                    }

                    if(goal_state_.isDone())
                    {
                        const auto& result_ptr = client_->getResult();
                        result_policy_.onNewMessage(*result_ptr, *this, "result");

                        goal_finished_ = true;
                    }

                    if(!goal_finished_) { setStatusRunningAndYield(); }
                }

                return GoalState2Status(goal_state_);
            }

            return BT::NodeStatus::FAILURE;;
        }

        virtual void halt() override
        {
            if(client_ && status() == BT::NodeStatus::RUNNING) {
                client_->cancelGoal();
                //Get result when cancelling
                const auto& result_ptr = client_->getResult();
                result_policy_.onNewMessage(*result_ptr, *this, "result");
		    }
            goal_finished_ = false;
            CoroActionNode::halt();
        }
    
    private:
        BT::NodeStatus GoalState2Status(const GoalState& _state)
        {
            switch(_state.state_)
            {
                case GoalState::StateEnum::PENDING:
                    return BT::NodeStatus::IDLE;
                    break;
                case GoalState::StateEnum::ACTIVE:
                case GoalState::StateEnum::RECALLED:
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

        void FeedbackCallback(const typename Feedback::ConstPtr& _feedback_msg)
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

        bool goal_finished_ { false };
        GoalState goal_state_ { GoalState::StateEnum::PENDING, "" };

        std::mutex        feedback_mutex_;
        std::atomic<bool> new_feedback_ { false };
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
