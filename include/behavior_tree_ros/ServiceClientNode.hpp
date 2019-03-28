#ifndef SERVICE_CLIENT_NODE_HPP
#define SERVICE_CLIENT_NODE_HPP

#include <topic_tools/shape_shifter.h>
#include <behaviortree_cpp/utils/demangle_util.h>

#include "ROSActionNode.hpp"
#include "details/serialization.hpp"

namespace BT_ROS
{
template <class MessageType>
class ServiceClientNode final : public ROSActionNode
{
    public:
        ServiceClientNode(const std::string& _name, const BT::NodeConfiguration& _config) : ROSActionNode(_name, _config)
        {
            shape_shifter_.morph(serialization::msgMD5Sum<typename MessageType::Request>(),
                                 serialization::msgDataType<typename MessageType::Request>(),
                                 serialization::msgDefinition<typename MessageType::Request>(), "" );

            const auto& service = getInput<std::string>("service");
            if(!service) { throw BT::RuntimeError { name() + ": " + service.error() }; }

            client_ = node_handle_.serviceClient<MessageType>(service.value());

        }
        ~ServiceClientNode() = default;

        static BT::PortsList providedPorts()
        {
            BT:: PortsList ports { BT::InputPort<std::string>("service", "ROS service name") };

            for(const auto& field : msgInfo().fields())
            {
                if(field.isConstant()) { continue; }
                //Setting void as the port type disables type checking
                const auto& field_port = BT::InputPort<void>(field.name(), std::string { "Auto-generated field from " }
                                                                            + BT::demangle(typeid(MessageType)));
                ports.insert(field_port);
            }

            return ports;
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);

            try
            {
                for(const auto& field : msgInfo().fields())
                {
                    serialization::serializeField(*this, field, serialization_buffer_);
                }

                ros::serialization::OStream stream(serialization_buffer_.data(), serialization_buffer_.size());
                shape_shifter_.read(stream);

                typename MessageType::Response service_response {};
                if(!this->client_.call(shape_shifter_, service_response, shape_shifter_.getMD5Sum())) { return BT::NodeStatus::FAILURE; }

                serialization_buffer_.clear();
                return BT::NodeStatus::SUCCESS;
            }
            catch(const std::out_of_range&)
            {
                throw BT::RuntimeError { name() + ": unrecognized field type in message " +  std::string { serialization::msgDataType<typename MessageType::Request>() }
                                            + ". Non-builtin types automatic serialization is not supported."
                                            + " Implement specializations for buildMessage<> and requiredMessageParameters<> functions instead." };
            }
        }

        virtual void halt() override {}

        static const RosIntrospection::ROSMessage& msgInfo() { static const auto msg_info = serialization::msgInfo<typename MessageType::Request>(); return msg_info; };

    private:
        ros::ServiceClient client_;
        topic_tools::ShapeShifter shape_shifter_;
        std::vector<uint8_t> serialization_buffer_;
};
}

#endif
