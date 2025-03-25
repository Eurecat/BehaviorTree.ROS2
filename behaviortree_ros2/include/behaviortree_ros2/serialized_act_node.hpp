
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/serialization_policies.hpp"
#include "behaviortree_ros2/deserialization_policies.hpp"
namespace BT
{
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
            {
                if(this->client_instance_->action_client && !this->client_instance_->action_client->action_server_is_ready())
                {
                    // force re-try connection on the first tick
                    this->action_name_should_be_checked_ = true;
                    this->action_name_ = "";
                }
            }

        //~SerializedActionClientNode(){ halt();}

        static BT::PortsList providedPorts()
        {
            PortsList provided_port_list =  RosActionNode<ActionType>::providedPorts();

            const auto& goal_ports = GoalDeserializationPolicy<typename ActionType::Goal>::requiredPorts();
            provided_port_list.insert(goal_ports.cbegin(), goal_ports.cend());

            const auto& result_ports = ResultSerializationPolicy<typename ActionType::Result>::requiredPorts("result");
            provided_port_list.insert(result_ports.cbegin(), result_ports.cend());

            const auto& feedback_ports = FeedbackSerializationPolicy<typename ActionType::Feedback>::requiredPorts("feedback");
            provided_port_list.insert(feedback_ports.cbegin(), feedback_ports.cend());

            provided_port_list.insert( OutputPort<std::string>("goal_state", "Goal Error State") );
            return provided_port_list;
        }

        bool setGoal(typename ActionType::Goal& goal) override
        {
            if(!goal_policy_.isParserInit() || this->action_name_ != prev_action_name_goal)
            {
                goal_policy_.initParser(this->action_name_,BT_ROS::msgName<typename ActionType::Goal>());
                prev_action_name_goal = this->action_name_;
            }
            const RosActionNode<ActionType>* rosactione_ptr = dynamic_cast<const RosActionNode<ActionType>*>(this);
            const BT::TreeNode* tree_node_ptr = dynamic_cast<const BT::TreeNode*>(rosactione_ptr);
            goal = goal_policy_.buildMessage(*tree_node_ptr);
            return true;
        }

        BT::NodeStatus onFeedback(const std::shared_ptr<const typename ActionType::Feedback> feedback) override
        {
            if(!feedback_policy_.isParserInit() || this->action_name_ != prev_action_name_feedback)
            {
                feedback_policy_.initParser(this->action_name_,BT_ROS::msgName<typename ActionType::Feedback>());
                prev_action_name_feedback = this->action_name_;
            }
            feedback_policy_.onNewMessage(std::const_pointer_cast<typename ActionType::Feedback>(feedback), *this, "feedback");
            return NodeStatus::RUNNING;
        }
        BT::NodeStatus onResultReceived(const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult& wr_result) override
        {
            if (wr_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                const auto & result = wr_result.result;
                //RCLCPP_INFO(this->logger(), "Action succeeded with result: %d", result->done);
                if(!result_policy_.isParserInit() || this->action_name_ != prev_action_name_result)
                {
                    result_policy_.initParser(this->action_name_,BT_ROS::msgName<typename ActionType::Result>());
                    prev_action_name_result = this->action_name_;
                }
                result_policy_.onNewMessage(result, *this, "result");
            } else {
                //RCLCPP_ERROR(this->logger(), "Action failed");
                //TODO: IS HERE A RESULT MESSAGE TO SHOW THAT WE SHOULD SERIALIZE????
                return NodeStatus::FAILURE;
            }
           
            return NodeStatus::SUCCESS;
        }

        BT::NodeStatus onFailure(ActionNodeErrorCode error) override
        {
            std::string error_str(toStr(error));
            this->setOutput("goal_state", error_str);
            RCLCPP_ERROR(this->logger(), "ACTION %s FAILED with error: %s",this->action_name_.c_str(), toStr(error));
            return BT::NodeStatus::FAILURE;
        }

        /*
        //TODO: Do we want to do something here??
        virtual void halt() override
        {
        }*/
    
    private:
        std::string prev_action_name_goal{""};
        std::string prev_action_name_feedback{""};
        std::string prev_action_name_result{""};
        GoalDeserializationPolicy<typename ActionType::Goal> goal_policy_ {};
        FeedbackSerializationPolicy<typename ActionType::Feedback> feedback_policy_ {};
        ResultSerializationPolicy<typename ActionType::Result> result_policy_ {};
};

//Shortcut alias
template <class ActionType>
using ActionClient = SerializedActionClientNode<ActionType, BT_ROS::NoDeserialization,
                                                              BT_ROS::NoSerialization,
                                                             BT_ROS::NoSerialization>; // ActionCall<ActionType>

template <class ActionType>
using AutoDesJsonSerActionClient = SerializedActionClientNode<ActionType, BT_ROS::AutomaticDeserialization,
                                                                       BT_ROS::JsonSerialization,
                                                                       BT_ROS::JsonSerialization>; // ActionAutoCallJson<ActionType>

template <class ActionType>
using AutoDesAutoSerActionClient = SerializedActionClientNode<ActionType, BT_ROS::AutomaticDeserialization,
                                                                        BT_ROS::AutomaticSerialization,
                                                                        BT_ROS::AutomaticSerialization>; // ActionAutoCallAuto<ActionType>

template <class ActionType>
using AutoDesSmartJsonSerActionClient = SerializedActionClientNode<ActionType, BT_ROS::AutomaticDeserialization,
                                                                       BT_ROS::SmartJsonSerialization,
                                                                       BT_ROS::SmartJsonSerialization>; // ActionAutoCallSmartJson<ActionType>
}