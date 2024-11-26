#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_ros2/serialization_policies.hpp"

using namespace BT_ROS;

namespace BT
{

    template <class MessageType, template <class> class SerializationPolicy>
    class SerializedSubNode : public RosTopicSubNode<MessageType> , public SerializationPolicy<MessageType>
    {
        public:
        SerializedSubNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
            : RosTopicSubNode<MessageType>(name, conf, params)
        {
           /* std::cout << "Creating sub for " << BT::demangle(typeid(MessageType)) << " - " << msgName<MessageType>() << " (vs std_msgs/String)\n" << std::flush;*/
            topic_type_ = msgName<MessageType>();
        }
        
        static BT::PortsList providedPorts()
        {
            //TODO: QUEUE SIZE NOT SUPPORTED
            PortsList provided_port_list =  RosTopicSubNode<MessageType>::providedPorts();

            provided_port_list.insert(BT::InputPort<bool>("consume_msgs", false, "Should messages be consumed?"));
            provided_port_list.insert(BT::InputPort<bool>("reinit", false, "Instantiate the subscriber at every new tick"));

            const auto& policy_ports = SerializationPolicy<MessageType>::requiredPorts();
            provided_port_list.insert(policy_ports.cbegin(), policy_ports.cend());

            return provided_port_list;
        }
        
        bool latchLastMessage() const override
        {
            return isLatch_;
        }
        void fetchSubscriberValues()
        {
            this->topic_name_ = this->template getInput<std::string>("topic_name").value_or(this->topic_name_);
            isLatch_ = this->template getInput<bool>("consume_msgs").value_or(false);
            reinit_ = this->template getInput<bool>("reinit").value_or(false);
        }
        NodeStatus onTick(const std::shared_ptr<MessageType>& last_msg) override
        {
            fetchSubscriberValues();

            if(!serialization_policy_.isParserInit() || this->topic_name_ != prev_topic_name_)
            {
                serialization_policy_.initParser(this->topic_name_,topic_type_);
                prev_topic_name_ = this->topic_name_;
            }

            if(last_msg)  // empty if no new message received, since the last tick
            {
                serialization_policy_.onNewMessage(last_msg,*this);
                if (reinit_)
                {
                    this->sub_instance_ = nullptr;
                }
            }
            else
                return NodeStatus::FAILURE;

            return NodeStatus::SUCCESS;
        }
        
        //SERIALIZATION POLICY
        SerializationPolicy<MessageType> serialization_policy_ {};
        std::string topic_type_;
        std::string prev_topic_name_{""};
        bool isLatch_{false};
        bool reinit_{false};
    };

    //Shortcut alias
    template <class MessageType>
    using Subscriber = SerializedSubNode<MessageType, BT_ROS::NoSerialization>;

    template <class MessageType>
    using SerializedSubscriber = SerializedSubNode<MessageType, BT_ROS::JsonSerialization>;

    template <class MessageType>
    using SmartSerializedSubscriber = SerializedSubNode<MessageType, BT_ROS::SmartJsonSerialization>;

    template <class MessageType>
    using SmartTimeEnabledSerializedSubscriber = SerializedSubNode<MessageType, BT_ROS::SmartTimeEnabledJsonSerialization>;
}