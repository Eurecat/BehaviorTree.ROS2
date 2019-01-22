#ifndef PUBLISHER_NODE_HPP
#define PUBLISHER_NODE_HPP

#include <topic_tools/shape_shifter.h>

#include "ROSActionNode.hpp"
#include "details/serialization.hpp"

namespace BT_ROS
{
template <class MessageType>
class BasePublisherNode : public ROSActionNode
{
    public:
        using ROSActionNode::ROSActionNode;
        virtual ~BasePublisherNode() = default;

        virtual void onInit() override
        {
            std::string topic;
            uint32_t queue_size;
            bool latch;

            if(!getParam("topic", topic))           { throw std::runtime_error { "Missing topic parameter" }; }
            if(!getParam("queue_size", queue_size)) { throw std::runtime_error { "Missing queue size parameter" }; }
            if(!getParam("latch", latch))           { throw std::runtime_error { "Missing latch parameter" }; }

            publisher_ = node_handle_.advertise<MessageType>(topic, queue_size, latch);
        }

        virtual void halt() override {}

    protected:
        ros::Publisher publisher_;
};

template <class MessageType, bool Serialize = true>
class PublisherNode;

template <class MessageType>
class PublisherNode<MessageType, false> final : public BasePublisherNode<MessageType>
{
    public:
        using BasePublisherNode<MessageType>::BasePublisherNode;
        ~PublisherNode() = default;

        static const NodeParameters& requiredNodeParameters()
        {
            static NodeParameters params { { "topic", "" }, { "queue_size", "1" }, { "latch", "false" } };
            
            const auto& message_parameters = requiredMessageParameters<MessageType>();
            params.insert(message_parameters.cbegin(), message_parameters.cend());

            return params;
        }

        virtual NodeStatus tick() override
        {
            try
            {
                const auto& message = buildMessage<MessageType>(*this);
                this->publisher_.publish(message);
            }
            catch(const std::runtime_error&)      { return NodeStatus::FAILURE; }
            catch(const BT::bad_optional_access&) { return NodeStatus::FAILURE; }

            return NodeStatus::SUCCESS;
        }
};

template <class MessageType>
class PublisherNode<MessageType, true> final : public BasePublisherNode<MessageType>
{
    public:
        PublisherNode(const std::string& _name, const NodeParameters& _params) : BasePublisherNode<MessageType>(_name, _params)
        {
            shape_shifter_.morph(serialization::msgMD5Sum<MessageType>(),
                                 serialization::msgDataType<MessageType>(),
                                 serialization::msgDefinition<MessageType>(), "" );
        }
        ~PublisherNode() = default;

        static const NodeParameters& requiredNodeParameters()
        {
            static NodeParameters params { { "topic", "" }, { "queue_size", "1" }, { "latch", "false" } };

            for(const auto& field : msgInfo().fields())
            {
                if(field.isConstant()) { continue; }
                params[field.name()] = "";
            }

            return params;
        }

        virtual NodeStatus tick() override
        {
            try
            {
                for(const auto& field : msgInfo().fields())
                {
                    serialization::serializeField(*this, field, serialization_buffer_);
                }

                ros::serialization::OStream stream(serialization_buffer_.data(), serialization_buffer_.size());
                shape_shifter_.read(stream);
                this->publisher_.publish(shape_shifter_);

                serialization_buffer_.clear();
            }
            catch(const std::runtime_error&)      { return NodeStatus::FAILURE; }
            catch(const BT::bad_optional_access&) { return NodeStatus::FAILURE; }
            catch(const std::out_of_range&)
            {
                throw std::runtime_error { "PublisherNode: unrecognized field type in message " +  std::string { serialization::msgDataType<MessageType>() }
                                            + ". Non-builtin types automatic serialization is not supported."
                                            + " Implement specializations for buildMessage<> and requiredMessageParameters<> functions instead." };
            }

            return NodeStatus::SUCCESS;
        }

    private:
        static const RosIntrospection::ROSMessage& msgInfo() { static const auto msg_info = serialization::msgInfo<MessageType>(); return msg_info; }

    private:
        topic_tools::ShapeShifter shape_shifter_;
        std::vector<uint8_t> serialization_buffer_;
};
}

#endif
