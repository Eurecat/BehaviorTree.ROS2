#ifndef BEHAVIOR_TREE_INTERCOM_NODE_HPP
#define BEHAVIOR_TREE_INTERCOM_NODE_HPP

#include <string>
#include <vector>
#include <atomic>
#include <mutex>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include "behavior_tree_ros/HandShakeAction.h"
#include "behavior_tree_ros/ExchangeInfoAction.h"

namespace BT_ROS
{
    class RosHandShake final
    {
        public:
            RosHandShake();
            ~RosHandShake() = default;

        private:
            void HandShakeFatherTopicCallback(const std_msgs::StringConstPtr& _topic_msg);
            void HandShakeChildTopicCallback(const std_msgs::StringConstPtr& _topic_msg);
            void HandShakeEndTopicCallback(const std_msgs::Bool::ConstPtr& _topic_msg);

            void HandShakeActionCallback(const behavior_tree_ros::HandShakeGoalConstPtr& _goal_msg);
            void ThreeWayHandshakeFather(const behavior_tree_ros::HandShakeGoalConstPtr& _goal_msg);
            void ThreeWayHandshakeChild(const behavior_tree_ros::HandShakeGoalConstPtr& _goal_msg);
            void HandShakeActionPreemptCallback();

        private:
            ros::NodeHandle public_node_handle_;
            ros::NodeHandle private_node_handle_ { "~" };

            ros::Publisher  send_father_signal_publisher_;
            ros::Publisher  send_child_signal_publisher_;
            ros::Publisher  send_end_signal_publisher_;
            ros::Subscriber get_father_signal_subscriber_;
            ros::Subscriber get_child_signal_subscriber_;
            ros::Subscriber get_end_signal_subscriber_;

            actionlib::SimpleActionServer<behavior_tree_ros::HandShakeAction> handshake_action_server_;
            behavior_tree_ros::HandShakeResult handshake_action_result_;

            std::atomic<bool> new_father_handshake_topic_msg_ { false };
            std::atomic<bool> new_child_handshake_topic_msg_ { false };
            std::atomic<bool> end_handshake_topic_msg_ { false };
            std::atomic<bool> action_cancelled_ {false};

            std::string handshake_father_topic_msg_;
            std::string handshake_child_topic_msg_;
    }; // class RosHandShake

    class RosExchangeInfo final
    {
        public:
            RosExchangeInfo();
            ~RosExchangeInfo() = default;

        private:
            void ExchangeInfoTopicCallback(const std_msgs::StringConstPtr& _topic_msg);

            void ExchangeInfoActionCallback(const behavior_tree_ros::ExchangeInfoGoalConstPtr& _goal_msg);
            void ExchangeInfoActionPreemptCallback();

        private:
            ros::NodeHandle public_node_handle_;
            ros::NodeHandle private_node_handle_ { "~" };

            ros::Publisher  send_info_publisher_;
            ros::Subscriber get_info_subscriber_;

            actionlib::SimpleActionServer<behavior_tree_ros::ExchangeInfoAction> exchange_info_action_server_;
            behavior_tree_ros::ExchangeInfoResult exchange_info_action_result_;

            std::atomic<bool> new_exchange_info_topic_msg_ { false };
            std::mutex exchange_info_mutex_;

            std::vector<std::string> exchange_info_topic_msgs_;
    }; // class RosExchangeInfo

    // class RosExchangeImage final
    // {
    //     public:
    //         RosExchangeImage();
    //         ~RosExchangeImage() = default;

    //     private:
    //         void ExchangeImageTopicCallback(const std_msgs::StringConstPtr& _topic_msg);

    //         void ExchangeImageActionCallback(const behavior_tree_ros::ExchangeImageGoalConstPtr& _goal_msg);
    //         void ExchangeImageActionPreemptCallback();

    //     private:
    //         ros::NodeHandle public_node_handle_;
    //         ros::NodeHandle private_node_handle_ { "~" };

    //         ros::Publisher  send_image_publisher_;
    //         ros::Subscriber get_image_subscriber_;

    //         actionlib::SimpleActionServer<behavior_tree_ros::ExchangeImageAction> exchange_image_action_server_;
    //         behavior_tree_ros::ExchangeImageResult exhange_image_action_result_;

    //         std::atomic<bool> new_exhange_image_topic_msg_ { false };
    //         std::mutex exhange_image_mutex_;

    //         std::vector<behavior_tree_ros::Image> exhange_image_topic_msgs_;
    // }; // class RosExchangeImage

} // namespace BT_ROS

#endif //BEHAVIOR_TREE_INTERCOM_NODE_HPP