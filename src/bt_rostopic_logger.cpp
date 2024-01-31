#include "bt_rostopic_logger.h"
#include "behavior_tree_ros/Transition.h"
#include "behavior_tree_ros/TreeExecutionStatus.h"
namespace BT_ROS
{
    //std::atomic<bool> RosTopicLogger::ref_count(false);

    RosTopicTransitionLogger::RosTopicTransitionLogger(const BT::Tree& tree, ros::Publisher pub) : BT::StatusChangeLogger(tree.rootNode()), bt_transition_publisher_(pub)
    {
    }
    RosTopicTransitionLogger::~RosTopicTransitionLogger()
    {
        //ref_count.store(false);
    }

    void RosTopicTransitionLogger::callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status,
                                BT::NodeStatus status)
    {
        //We only publish transitions for action nodes to avoid flooding the channel and ease debugging
        if(node.type()!=BT::NodeType::ACTION)
        return;

        behavior_tree_ros::Transition msg;

        msg.uid = node.UID();
        msg.name = node.name();
        msg.model = node.registrationName();
        msg.status = ConvertStatusToString(status);
        msg.prev_status = ConvertStatusToString(prev_status);
        bt_transition_publisher_.publish(msg);
    }

    void RosTopicTransitionLogger::flush()
    {
        //ref_count = false;
    }

    std::string RosTopicTransitionLogger::ConvertStatusToString(BT::NodeStatus status)
    {
        switch(status)
        {
            case BT::NodeStatus::IDLE:
                return "IDLE";
                break;
            case BT::NodeStatus::RUNNING:
                return "RUNNING";
                break;
            case BT::NodeStatus::SUCCESS:
                return "SUCCESS";
                break;
            case BT::NodeStatus::FAILURE:
                return "FAILURE";
                break;
            case BT::NodeStatus::SKIPPED:
                return "SKIPPED";
                break;
            case BT::NodeStatus::PAUSED:
                return "PAUSED";
                break;
            default:
                return "IDLE";
                break;
        }
    }

    RosTopicStatusLogger::RosTopicStatusLogger(const BT::Tree& tree, ros::Publisher pub, unsigned int tree_uid, std::string tree_name, std::string tree_file_name, ros::Time start_time) : BT::StatusChangeLogger(tree.rootNode()), bt_execution_status_publisher_(pub)
    {
        tree_filename_ = tree_file_name;
        tree_uid_ = tree_uid;
        execution_time_ = start_time;
        tree_name_ = tree_name;
    }
    RosTopicStatusLogger::~RosTopicStatusLogger()
    {
        //ref_count.store(false);
    }

    void RosTopicStatusLogger::callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status,
                                BT::NodeStatus status)
    {
        if (status == BT::NodeStatus::PAUSED)
        {
            behavior_tree_ros::TreeExecutionStatus status_msg {};
            status_msg.time_start = execution_time_;
            status_msg.name = tree_name_;
            status_msg.time = ros::Time::now();
            status_msg.uid = tree_uid_;
            status_msg.file = tree_filename_;
            status_msg.status  = "PAUSED";
            was_paused = true;
            bt_execution_status_publisher_.publish(status_msg);
        }
        else if (was_paused)
        {
            behavior_tree_ros::TreeExecutionStatus status_msg {};
            status_msg.time_start = execution_time_;
            status_msg.name = tree_name_;
            status_msg.time = ros::Time::now();
            status_msg.uid = tree_uid_;
            status_msg.file = tree_filename_;
            status_msg.status  = "RUNNING";
            was_paused = false;
            bt_execution_status_publisher_.publish(status_msg);
        }
    }

    void RosTopicStatusLogger::flush()
    {
        //ref_count = false;
    }
}   // end namespace
