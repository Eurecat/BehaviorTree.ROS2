#include <string>
#include <vector>
#include <atomic>
#include <mutex>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <std_msgs/String.h>

#include "behavior_tree_ros/HandShakeAction.h"

class RosHandshake
{
public:
    RosHandshake() :
        handshake_action_server_ (public_node_handle_, "handshake_send_signal", boost::bind(&RosHandshake::HandshakeActionCallback, this, _1), false)
    {
        // Get intercom topic name
        std::string handshake_topic_name = private_node_handle_.param<std::string>("handshake_topic_name", "/remote/bt_intercom");

        // Publisher
        send_signal_publisher_ = public_node_handle_.advertise<std_msgs::String>(handshake_topic_name, 1, true);

        // Subscriber
        get_signal_subscriber_ = public_node_handle_.subscribe(handshake_topic_name, 10, &RosHandshake::HandShakeTopicCallback, this);

        // Actionlib
        handshake_action_server_.registerPreemptCallback(boost::bind(&RosHandshake::HandShakeActionPreemptCallback, this));
        handshake_action_server_.start();
    }
    ~RosHandshake() = default;

    void HandShakeTopicCallback(const std_msgs::StringConstPtr& _topic_msg)
    {
        ROS_INFO("Received topic message! [%s]", _topic_msg->data.c_str());
        std::unique_lock<std::mutex> lock (handshake_mutex_);
        handshake_topic_msgs_.emplace_back(_topic_msg->data);
        new_handshake_topic_msg_ = true;
    }

    void HandshakeActionCallback(const behavior_tree_ros::HandShakeGoalConstPtr& _goal_msg)
    {
        ROS_INFO("Starting Handshake Action callback! [%s] [%s]", _goal_msg->bt_id.c_str(), _goal_msg->message.c_str());
        bool signal_sent = false;
        bool signal_received = false;

        try
        {
            while(!signal_received)
            {
                // Check signal received is the correct one
                {
                    std::unique_lock<std::mutex> lock (handshake_mutex_);

                    if(new_handshake_topic_msg_)
                    {
                        new_handshake_topic_msg_ = false;
                        std::string handshake_bt_id {""};
                        std::string handshake_message {""};
                        for (auto it = handshake_topic_msgs_.begin(); it != handshake_topic_msgs_.end(); it++)
                        {
                            // ID and message are separated by ":"
                            size_t pos = it->find_first_of(":");
                            handshake_bt_id = it->substr(0, pos);
                            handshake_message = it->substr(pos+1, it->size());
                            // Checking if msg received is not empty, is from another BT and is in the same stage
                            if(!it->empty() && handshake_bt_id != _goal_msg->bt_id && handshake_message == _goal_msg->message)
                            {
                                ROS_INFO("Message received from another node [%s]", it->c_str());
                                // Remove matched message
                                handshake_topic_msgs_.erase(it--);
                                signal_received = true;

                                break; // Stop checking msgs as we found what we were looking for
                            }
                            // Remove unmatched message
                            handshake_topic_msgs_.erase(it--);
                        }
                    }
                }

                // Send signal only one time
                if(!signal_sent)
                {
                    ROS_INFO("Sending signal to the other node!");
                    std_msgs::String msg_to_send;
                    msg_to_send.data = _goal_msg->bt_id + ":" + _goal_msg->message;
                    send_signal_publisher_.publish(msg_to_send);
                    signal_sent = true;
                }

                usleep(2e5); // not overload CPU
            }

            if(handshake_action_server_.isActive())
            {
                ROS_INFO("Handshake succeeded!!");
                handshake_action_result_.result = true;
                handshake_action_server_.setSucceeded(handshake_action_result_, "Synchronization succeeded!");
            }
        }
        catch(const std::runtime_error& ex)
        {
            ROS_ERROR("Error in Handshake Action: %s", ex.what());
            handshake_action_result_.result = false;
            handshake_action_server_.setAborted(handshake_action_result_, "Synchronization aborted!");
        }
    }

    void HandShakeActionPreemptCallback()
    {
        ROS_INFO("Action Goal canceled!");
        handshake_action_result_.result = false;
        handshake_action_server_.setPreempted(handshake_action_result_, "Goal preempted");
    }

private:
    ros::NodeHandle public_node_handle_;
    ros::NodeHandle private_node_handle_ { "~" };

    ros::Publisher  send_signal_publisher_;
    ros::Subscriber get_signal_subscriber_;

    actionlib::SimpleActionServer<behavior_tree_ros::HandShakeAction> handshake_action_server_;
    behavior_tree_ros::HandShakeResult handshake_action_result_;

    std::atomic<bool> new_handshake_topic_msg_ { false };
    std::mutex handshake_mutex_;

    std::vector<std::string> handshake_topic_msgs_;
}; // class RosHandshake

int main(int argc, char **argv)
{
    ros::init(argc, argv, "handshake_signal_node");

    RosHandshake handshake;

    ros::spin();

    return 0;
}