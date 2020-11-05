#ifndef LOOKUP_TRANSFORM_NODE_HPP
#define LOOKUP_TRANSFORM_NODE_HPP

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <behaviortree_cpp_v3/action_node.h>

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
                     BT::OutputPort<tf::StampedTransform>("output", "Result transform")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);
            const auto& source_frame      = getInput<std::string>("source_frame");
            const auto& target_frame      = getInput<std::string>("target_frame");
            const auto use_last_available = getInput<bool>("use_last_available");

            if(!source_frame)       { throw BT::RuntimeError { name() + ": " + source_frame.error() }; }
            if(!target_frame)       { throw BT::RuntimeError { name() + ": " + target_frame.error() }; }
            if(!use_last_available) { throw BT::RuntimeError { name() + ": " + use_last_available.error() }; }

            try
            {
                tf::StampedTransform transform;

                const auto& transform_time = use_last_available.value() ? ros::Time(0) : ros::Time::now();
                tf_listener_.lookupTransform(target_frame.value(), source_frame.value(), transform_time, transform);

		//Workaround to deal with old tf when using last available
		if((ros::Time::now() - transform.stamp_) > ros::Duration(1.0))
		{
			std::cout << "TF too old!" << std::endl;
			return BT::NodeStatus::FAILURE;
		}

                setOutput("output", transform);
                return BT::NodeStatus::SUCCESS;
            }
            catch(const tf::TransformException&)  { return BT::NodeStatus::FAILURE; }
        }

    private:
        tf::TransformListener tf_listener_;
};
}

#endif
