#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "behaviortree_ros2/deserialization_policies.hpp"

namespace BT
{

    template <class MessageType, template <class> class DeserializationPolicy>
    class SerializedPubNode : public RosTopicPubNode<MessageType> , public DeserializationPolicy<MessageType>
    {
        public:
            SerializedPubNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) 
            : RosTopicPubNode<MessageType>(name, conf, params )
            {
                topic_type_ = msgName<MessageType>();
            }
            ~SerializedPubNode() = default;

            static BT::PortsList providedPorts()
            {
               // std::cout << "ProvidedPorts in SerializedPubNode for " << BT::demangle(typeid(MessageType)) << "\n" << std::flush;
                PortsList provided_port_list =  RosTopicPubNode<MessageType>::providedPorts();

                const auto& policy_ports = DeserializationPolicy<MessageType>::requiredPorts();
                provided_port_list.insert(policy_ports.cbegin(), policy_ports.cend());

                return provided_port_list;
            }

            bool setMessage(MessageType& msg)  override
            {
                try
                {
                    if(!deserialization_policy_.isParserInit() || topic_name_ != this->prev_topic_name_)
                    {
                        deserialization_policy_.initParser(this->prev_topic_name_,topic_type_);
                        topic_name_ = this->prev_topic_name_;
                    }
                    msg = deserialization_policy_.buildMessage(*this);
                }
                catch(const std::out_of_range&)
                {
                    return false;
                }
                return true;
            }

        private:
            DeserializationPolicy<MessageType> deserialization_policy_ {};
            std::string topic_type_;
            std::string topic_name_{""};
    };
    //Shortcut alias
    template <class MessageType>
    using Publisher = SerializedPubNode<MessageType, BT_ROS::NoDeserialization>;

    template <class MessageType>
    using AutomaticPublisher = SerializedPubNode<MessageType, BT_ROS::AutomaticDeserialization>;
    
}