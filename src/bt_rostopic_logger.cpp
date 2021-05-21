#include "bt_rostopic_logger.h"

#include <std_msgs/String.h>

namespace BT_ROS
{
std::atomic<bool> RosTopicLogger::ref_count(false);

RosTopicLogger::RosTopicLogger(const BT::Tree& tree, ros::Publisher pub) : BT::StatusChangeLogger(tree.rootNode()), bt_status_publisher_(pub)
{
    // It should be ok to have more than one rostopic logger
    // The user should be responsible of using different topics for each tree if wanted

    // bool expected = false;
    // if (!ref_count.compare_exchange_strong(expected, true))
    // {
    //     throw BT::LogicError("Only one instance of RosTopicLogger shall be created");
    // }
}
RosTopicLogger::~RosTopicLogger()
{
    ref_count.store(false);
}

void RosTopicLogger::callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status,
                             BT::NodeStatus status)
{
    using namespace std::chrono;

   // constexpr const char* whitespaces = "                         ";
    //constexpr const size_t ws_count = 25;

    //We only publish for action nodes to avoid flooding the channel and ease debugging
    if(node.type()!=BT::NodeType::ACTION)
	return;

    std_msgs::String msg;

    std::stringstream ss;

    ss << node.name().c_str() << ": " << toStr(status, true).c_str();

    msg.data = ss.str();

    bt_status_publisher_.publish(msg);

    /*double since_epoch = duration<double>(timestamp).count();
    printf("[%.3f]: %s%s %s -> %s",
           since_epoch, node.name().c_str(),
           &whitespaces[std::min(ws_count, node.name().size())],
           toStr(prev_status, true).c_str(),
           toStr(status, true).c_str() );
    std::cout << std::endl;*/
}

void RosTopicLogger::flush()
{
    //std::cout << std::flush;
	ref_count = false;
}

}   // end namespace
