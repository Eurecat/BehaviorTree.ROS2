#ifndef LOOKUP_TRANSFORM_NODE_HPP
#define LOOKUP_TRANSFORM_NODE_HPP

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/action_node.h"
#include "tf2_ros/buffer.h"

namespace BT_ROS
{
class LookupTransformNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~LookupTransformNode() = default;

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<std::string>("source_frame", "Origin TF frame"),
                     BT::InputPort<std::string>("target_frame", "Target TF frame"),
                     BT::InputPort<bool>("use_last_available", false, "Use last available transform?"),
                     BT::OutputPort<geometry_msgs::msg::TransformStamped>("output", "Result transform")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            const auto& source_frame      = getInput<std::string>("source_frame");
            const auto& target_frame      = getInput<std::string>("target_frame");
            const auto use_last_available = getInput<bool>("use_last_available");

            if(!source_frame)       { throw BT::RuntimeError { name() + ": " + source_frame.error() }; }
            if(!target_frame)       { throw BT::RuntimeError { name() + ": " + target_frame.error() }; }
            if(!use_last_available) { throw BT::RuntimeError { name() + ": " + use_last_available.error() }; }

            try
            {
                const auto& transform_time = use_last_available.value() ? rclcpp::Time(0) : rclcpp::Time();

                tf2_ros::Buffer tf_buffer(rclcpp::Clock::make_shared());
                geometry_msgs::msg::TransformStamped transform = tf_buffer.lookupTransform(target_frame.value(), source_frame.value(), transform_time);

		        //Workaround to deal with old tf when using last available
                if ((rclcpp::Clock().now() - transform.header.stamp) > rclcpp::Duration::from_seconds(1.0))
                {
                    std::cout << "TF too old!" << std::endl;
                    return BT::NodeStatus::FAILURE;
                }

                setOutput("output", transform);
                return BT::NodeStatus::SUCCESS;
            }
            catch(const tf2::TransformException& ex)  { return BT::NodeStatus::FAILURE; }
        }
};
}

#endif
