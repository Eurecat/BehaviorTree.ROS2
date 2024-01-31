#ifndef BT_ROSTOPIC_LOGGER_H
#define BT_ROSTOPIC_LOGGER_H

#include <cstring>
#include <behaviortree_cpp_v3/loggers/abstract_logger.h>
#include <ros/ros.h>

namespace BT_ROS
{
/**
 * @brief AddStdCoutLoggerToTree. Give  the root node of a tree,
 * a simple callback is subscribed to any status change of each node.
 *
 *
 * @param root_node
 * @return Important: the returned shared_ptr must not go out of scope,
 *         otherwise the logger is removed.
 */

class RosTopicTransitionLogger : public BT::StatusChangeLogger
{
    static std::atomic<bool> ref_count;

    //ros::NodeHandle nh_;
    ros::Publisher bt_transition_publisher_;

  public:
    RosTopicTransitionLogger(const BT::Tree& tree, ros::Publisher pub);
    ~RosTopicTransitionLogger() override;

    virtual void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status,
                          BT::NodeStatus status) override;

    virtual void flush() override;
    std::string ConvertStatusToString(BT::NodeStatus status);
};
class RosTopicStatusLogger : public BT::StatusChangeLogger
{
    static std::atomic<bool> ref_count;

    ros::Publisher bt_execution_status_publisher_;

  public:
    RosTopicStatusLogger(const BT::Tree& tree, ros::Publisher pub, unsigned int tree_uid, std::string tree_name, std::string tree_file_name, ros::Time start_time);
    ~RosTopicStatusLogger() override;

    virtual void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status,
                          BT::NodeStatus status) override;

    virtual void flush() override;
    std::string ConvertStatusToString(BT::NodeStatus status);
    
    bool was_paused {false};
    std::string tree_filename_ {};
    std::string tree_name_ {};
    ros::Time execution_time_;
    unsigned int tree_uid_;
};
}   // end namespace

#endif   // BT_COUT_LOGGER_H
