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

class RosTopicLogger : public BT::StatusChangeLogger
{
    static std::atomic<bool> ref_count;

    //ros::NodeHandle nh_;
    ros::Publisher bt_status_publisher_;
    bool status_paused {false};

  public:
    RosTopicLogger(const BT::Tree& tree, ros::Publisher pub);
    ~RosTopicLogger() override;

    virtual void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status,
                          BT::NodeStatus status) override;

    virtual void flush() override;
};

}   // end namespace

#endif   // BT_COUT_LOGGER_H
