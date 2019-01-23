#ifndef SERVICE_CLIENT_NODE_HPP
#define SERVICE_CLIENT_NODE_HPP

#include <topic_tools/shape_shifter.h>

#include "ROSActionNode.hpp"
#include "details/serialization.hpp"

namespace BT_ROS
{
template <class MessageType>
class BaseServiceClientNode : public ROSActionNode
{
    public:
        using ROSActionNode::ROSActionNode;
        virtual ~BaseServiceClientNode() = default;

        virtual void onInit() override
        {
            std::string service;
            if(!getParam("service", service)) { throw std::runtime_error { "Missing service name" }; }
            client_ = node_handle_.serviceClient<MessageType>(service);
        }

        virtual void halt() override {}
        
    protected:
        ros::ServiceClient client_;
};

template <class MessageType, bool Serialize = true>
class ServiceClientNode;

template <class MessageType>
class ServiceClientNode<MessageType, false> final : public BaseServiceClientNode<MessageType>
{
    public:
        using BaseServiceClientNode<MessageType>::BaseServiceClientNode;
        ~ServiceClientNode() = default;

        static const NodeParameters& requiredNodeParameters()
        {
            static NodeParameters params { { "service", "" } };

            const auto& message_parameters = requiredMessageParameters<MessageType>();
            params.insert(message_parameters.cbegin(), message_parameters.cend());

            return params;
        }

        virtual BT::NodeStatus tick() override
        {
            try
            {
                auto message = buildMessage<MessageType>(*this);
                if(!this->client_.call(message)) { return NodeStatus::FAILURE; }
            }
            catch(const std::runtime_error&)      { return NodeStatus::FAILURE; }
            catch(const BT::bad_optional_access&) { return NodeStatus::FAILURE; }

            return NodeStatus::SUCCESS;
        }
};

template <class MessageType>
class ServiceClientNode<MessageType, true> final : public BaseServiceClientNode<MessageType>
{
    public:
        ServiceClientNode(const std::string& _name, const NodeParameters& _params) : BaseServiceClientNode<MessageType>(_name, _params)
        {
            shape_shifter_.morph(serialization::msgMD5Sum<typename MessageType::Request>(),
                                 serialization::msgDataType<typename MessageType::Request>(),
                                 serialization::msgDefinition<typename MessageType::Request>(), "" );
        }
        ~ServiceClientNode() = default;

        static const NodeParameters& requiredNodeParameters()
        {
            static NodeParameters params { { "service", "" } };

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

                typename MessageType::Response service_response {};
                if(!this->client_.call(shape_shifter_, service_response, shape_shifter_.getMD5Sum())) { return NodeStatus::FAILURE; }

                serialization_buffer_.clear();
            }
            catch(const std::runtime_error&)      { return NodeStatus::FAILURE; }
            catch(const BT::bad_optional_access&) { return NodeStatus::FAILURE; }
            catch(const std::out_of_range&)
            {
                throw std::runtime_error { "ServiceClientNode: unrecognized field type in message " +  std::string { serialization::msgDataType<typename MessageType::Request>() }
                                            + ". Non-builtin types automatic serialization is not supported."
                                            + " Implement specializations for buildMessage<> and requiredMessageParameters<> functions instead." };
            }

            return NodeStatus::SUCCESS;
        }

        static const RosIntrospection::ROSMessage& msgInfo() { static const auto msg_info = serialization::msgInfo<typename MessageType::Request>(); return msg_info; };

    private:
        topic_tools::ShapeShifter shape_shifter_;
        std::vector<uint8_t> serialization_buffer_;
};
}

#endif
