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

#include "behavior_tree_ros/HandShake.h"
#include "behavior_tree_ros/HandShakeData.h"
#include "behavior_tree_ros/PerformHandShakeAction.h"
#include "behavior_tree_ros/PerformHandShakeDataAction.h"
#include "behavior_tree_ros/ExchangeInfoAction.h"

namespace BT_ROS
{
    class RosHandShake final
    {
        public:
            RosHandShake();
            ~RosHandShake() = default;

        private:
            void ThreeWayHandShakeTopicCallbackClient(const behavior_tree_ros::HandShake& _topic_msg);
            void ThreeWayHandShakeTopicCallbackServer(const behavior_tree_ros::HandShake& _topic_msg);

            void ThreeWayHandShakeActionCallback(const behavior_tree_ros::PerformHandShakeGoalConstPtr& _goal_msg);
            void ThreeWayHandshakeServer(const behavior_tree_ros::PerformHandShakeGoalConstPtr& _goal_msg);
            void ThreeWayHandshakeClient(const behavior_tree_ros::PerformHandShakeGoalConstPtr& _goal_msg);
            void ThreeWayHandShakeActionPreemptCallback();
            void PublishSmsCallback(const ros::TimerEvent& ev);

        private:
            ros::NodeHandle public_node_handle_;
            ros::NodeHandle private_node_handle_ { "~" };

            ros::Publisher  sync_publisher_;
            ros::Subscriber sync_subscriber_;

            actionlib::SimpleActionServer<behavior_tree_ros::PerformHandShakeAction> handshake_action_server_;
            behavior_tree_ros::PerformHandShakeResult handshake_result_;

            std::atomic<bool> sync_received_ {false};
            std::atomic<bool> ack_received_ {false};
            std::atomic<bool> action_cancelled_ {false};

            int16_t my_seq_id_{0};

            std::string handshake_mode_;

            ros::Timer pub_timer_;

            double pub_period_s_ {1.0};

            behavior_tree_ros::HandShake  msg_to_send_;
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
    }; 
    
    class RosHandShakeData final
    {
        public:
            RosHandShakeData();
            ~RosHandShakeData() = default;

        private:
            void ThreeWayHandShakeDataTopicCallbackClient(const behavior_tree_ros::HandShakeData& _topic_msg);
            void ThreeWayHandShakeDataTopicCallbackServer(const behavior_tree_ros::HandShakeData& _topic_msg);

            void ThreeWayHandShakeDataActionCallback(const behavior_tree_ros::PerformHandShakeDataGoalConstPtr& _goal_msg);
            void ThreeWayHandshakeDataServer(const behavior_tree_ros::PerformHandShakeDataGoalConstPtr& _goal_msg);
            void ThreeWayHandshakeDataClient(const behavior_tree_ros::PerformHandShakeDataGoalConstPtr& _goal_msg);
            void ThreeWayHandShakeDataActionPreemptCallback();
            void PublishSmsCallback(const ros::TimerEvent& ev);

        private:
            ros::NodeHandle public_node_handle_;
            ros::NodeHandle private_node_handle_ { "~" };

            ros::Publisher  sync_publisher_;
            ros::Subscriber sync_subscriber_;

            actionlib::SimpleActionServer<behavior_tree_ros::PerformHandShakeDataAction> handshakedata_action_server_;
            behavior_tree_ros::PerformHandShakeDataResult handshakedata_result_;

            std::atomic<bool> sync_received_ {false};
            std::atomic<bool> ack_received_ {false};
            std::atomic<bool> action_cancelled_ {false};

            int16_t my_seq_id_{0};
            std::string data_to_send;
            std::string data_rx;
            std::string handshake_mode_;

            ros::Timer pub_timer_;

            double pub_period_s_ {1.0};

            behavior_tree_ros::HandShakeData  msg_to_send_;
    }; // class RosHandShake
    // class RosExchangeInfo

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