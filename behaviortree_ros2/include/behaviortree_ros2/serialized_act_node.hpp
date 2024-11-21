
#include "behaviortree_ros2/bt_action_node.hpp"

namespace BT
{

   /* enum ClientStatus
    {
        INVALID = 0,
        NOT_INSTANTIATED = 1,
        WAITING_CONN = 2,
        WAITING_RES = 3,
        RECEIVED_RES = 4
    };*/



    template <class ActionType,  template <class> class GoalDeserializationPolicy,
                                template <class> class ResultSerializationPolicy,
                                template <class> class FeedbackSerializationPolicy>
    class SerializedActionClientNode final : public RosActionNode<ActionType>,
                                        public GoalDeserializationPolicy<typename ActionType::Goal>,
                                        public ResultSerializationPolicy<typename ActionType::Result>,
                                        public FeedbackSerializationPolicy<typename ActionType::Feedback>
    {
    public:
        SerializedActionClientNode(const std::string& _name, const BT::NodeConfig& conf, const RosNodeParams& params) :
         RosActionNode<ActionType>(_name, conf, params)
           // client_status_(ClientStatus::NOT_INSTANTIATED)
        {
           // instantiateClient(false);
        }

        //~SerializedActionClientNode(){ halt();}

        static BT::PortsList providedPorts()
        {
            PortsList provided_port_list =  RosActionNode<ActionType>::providedPorts();
            
            provided_port_list.insert(BT::OutputPort<typename ActionType::Goal>("state", "ROSACTION state"));

            const auto& goal_ports = GoalDeserializationPolicy<typename ActionType::Goal>::requiredPorts();
            provided_port_list.insert(goal_ports.cbegin(), goal_ports.cend());

            const auto& feedback_ports = FeedbackSerializationPolicy<typename ActionType::Feedback>::requiredPorts("feedback");
            provided_port_list.insert(feedback_ports.cbegin(), feedback_ports.cend());

            const auto& result_ports = ResultSerializationPolicy<typename ActionType::Result>::requiredPorts("result");
            provided_port_list.insert(result_ports.cbegin(), result_ports.cend());

            return provided_port_list;
        }

        bool setGoal(typename ActionType::Goal& goal) override
        {
            goal = goal_policy_.buildMessage(*this);
            return false;
        }
        BT::NodeStatus onResultReceived(const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult& result) override
        {
            if (result.result->done)
            {
             //   result_policy_.onNewMessage(*result, *this, "result");
            }
            return result.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        }
        BT::NodeStatus onFeedback(const std::shared_ptr<const typename ActionType::Feedback> feedback) override
        {
            // Get state, save it in the output and save it in the output variable
            //goal_state_ = client_->getState();
            //setOutput("state", goal_state_);
            feedback_policy_.onNewMessage(std::const_pointer_cast<typename ActionType::Feedback>(feedback), *this, "feedback");

            return NodeStatus::RUNNING;
        }
        BT::NodeStatus onFailure(ActionNodeErrorCode error) override
        {
            RCLCPP_ERROR(this->logger(), "ACTION %s FAILED with error: %s",this->action_name_.c_str(), toStr(error));
            return BT::NodeStatus::FAILURE;
        }

        /*virtual BT::NodeStatus tick() override
        {
            instantiateClient(true);
            
            // NON blocking wait for server
            if(!client_->isServerConnected() && client_status_ == ClientStatus::WAITING_CONN && 
                (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - client_instantiated_time_) < std::chrono::milliseconds{WAIT_SRV_CONN_TIMEOUT_MS}))
            {    
                return BT::NodeStatus::RUNNING;
            }
            
            if (client_->isServerConnected())
            {
                {
                    const auto& goal_msg = goal_policy_.buildMessage(*this);
                    client_->sendGoal(goal_msg, {}, {}, boost::bind(&SimpleActionClientNode::FeedbackCallback, this, _1));
                    client_status_ = ClientStatus::WAITING_RES;
                }

                while(client_status_ == ClientStatus::WAITING_RES)
                {
                    // Check connection to prevent lock if server dies processing goal
                    if (!client_->isServerConnected()) 
                    {
                        client_status_ = ClientStatus::NOT_INSTANTIATED; // reinit on a later tick 
                        return BT::NodeStatus::FAILURE; 
                    }

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

                        client_status_ = ClientStatus::RECEIVED_RES;
                    }

                    if(client_status_ != ClientStatus::RECEIVED_RES) { setStatusRunningAndYield(); }
                }

                const auto bt_status = GoalState2Status(goal_state_);
                if(bt_status != BT::NodeStatus::RUNNING) 
                {
                    client_status_ = ClientStatus::NOT_INSTANTIATED; // reinit on a later tick 
                }
                return bt_status;
            }
            
            client_status_ = ClientStatus::NOT_INSTANTIATED; // reinit on a later tick 
            return BT::NodeStatus::FAILURE;
        }

        virtual void halt() override
        {
            if(client_ && status() == BT::NodeStatus::RUNNING) {
                client_->cancelGoal();
                //Get result when cancelling
                const auto& result_ptr = client_->getResult();
                result_policy_.onNewMessage(*result_ptr, *this, "result");
		    }
            client_status_ = ClientStatus::NOT_INSTANTIATED; // reinit on a later tick 
            CoroActionNode::halt();
        }*/
    
    private:
       /* BT::NodeStatus GoalState2Status(const GoalState& _state)
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
        }*/

        GoalDeserializationPolicy<typename ActionType::Goal>     goal_policy_     {};
        ResultSerializationPolicy<typename ActionType::Result>   result_policy_   {};
        FeedbackSerializationPolicy<typename ActionType::Feedback> feedback_policy_ {};

};

//Shortcut alias
template <class ActionType>
using SimpleActionClient = SerializedActionClientNode<ActionType, BT_ROS::NoDeserialization,
                                                              BT_ROS::NoSerialization,
                                                             BT_ROS::NoSerialization>;

template <class ActionType>
using AutomaticSimpleActionClient = SerializedActionClientNode<ActionType, BT_ROS::AutomaticDeserialization,
                                                                       BT_ROS::JsonSerialization,
                                                                       BT_ROS::JsonSerialization>;


template <class ActionType>
using AutomaticSmartSimpleActionClient = SerializedActionClientNode<ActionType, BT_ROS::AutomaticDeserialization,
                                                                       BT_ROS::SmartJsonSerialization,
                                                                       BT_ROS::SmartJsonSerialization>;
}
