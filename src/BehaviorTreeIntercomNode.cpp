#include "BehaviorTreeIntercomNode.hpp"

namespace BT_ROS
{
    RosHandShake::RosHandShake() :
        handshake_action_server_ (public_node_handle_, "behavior_tree/handshake", boost::bind(&RosHandShake::HandShakeActionCallback, this, _1), false)
    {
        // Get intercom topic name
        std::string handshake_topic_name = private_node_handle_.param<std::string>("handshake_topic_name", "/remote/bt_handshake");        
        // Publisher
        send_signal_publisher_ = public_node_handle_.advertise<std_msgs::String>(handshake_topic_name, 1, true);  
        send_handshake_end_signal_publisher_ = public_node_handle_.advertise<std_msgs::Bool>("/encouraging_mediator/handshake_end_signal", 1, false);    
        // Subscriber
        get_signal_subscriber_ = public_node_handle_.subscribe(handshake_topic_name, 10, &RosHandShake::HandShakeTopicCallback, this);   
        get_end_handshake_signal_subscriber_ = public_node_handle_.subscribe("/encouraging_mediator/handshake_end_signal", 10, &RosHandShake::HandShakeEndTopicCallback, this);     
        // Actionlib
        handshake_action_server_.registerPreemptCallback(boost::bind(&RosHandShake::HandShakeActionPreemptCallback, this));
        handshake_action_server_.start();
    }    
    
    void RosHandShake::HandShakeTopicCallback(const std_msgs::StringConstPtr& _topic_msg)
    {
        ROS_INFO("Received end Handshake topic message! [%s]", _topic_msg->data.c_str());
        std::unique_lock<std::mutex> lock (handshake_mutex_);
        handshake_topic_msgs_.emplace_back(_topic_msg->data);
        new_handshake_topic_msg_ = true;
    }   
 
    void RosHandShake::HandShakeEndTopicCallback(const std_msgs::Bool::ConstPtr& _topic_msg)
    {
        ROS_INFO("Received Handshake topic message! [%i]", _topic_msg->data);
        std::unique_lock<std::mutex> lock (handshake_mutex_);
        end_handshake_topic_msg_ = _topic_msg->data;
    }  
    
    void RosHandShake::ThreeWayHandshakeJapan(const behavior_tree_ros::HandShakeGoalConstPtr& _goal_msg){
        
        bool first_sync_sent = false;

        // Send handshake message to the other side every 0.5 second until we receive the ACK signal
        while(!first_sync_sent){
            // Send handshake message every 0.5 seconds 
            std_msgs::String msg_to_send;
            msg_to_send.data = _goal_msg->bt_id + ":" + _goal_msg->message + "_1";
            send_signal_publisher_.publish(msg_to_send);
            usleep(10e5);

            // Check if we have receive the first ACK signal from the other side
            if(new_handshake_topic_msg_){
                new_handshake_topic_msg_ = false;
                std::string handshake_bt_id {""};
                std::string handshake_message {""};
                for (auto it = handshake_topic_msgs_.begin(); it != handshake_topic_msgs_.end(); it++){
                    // ID and message are separated by ":"
                    size_t pos = it->find_first_of(":");
                    handshake_bt_id = it->substr(0, pos);
                    handshake_message = it->substr(pos+1, it->size());
                    // Checking if msg received is not empty, is from another BT and is in the same stage
                    if(!it->empty() && handshake_bt_id != _goal_msg->bt_id && handshake_message == _goal_msg->message+"_ack")
                    {
                        ROS_INFO("Message received from another node [%s]", it->c_str());
                        // Remove matched message
                        handshake_topic_msgs_.erase(it--);
                        first_sync_sent = true;                                
                        break; // Stop checking msgs as we found what we were looking for
                    }
                    // Remove unmatched message
                    handshake_topic_msgs_.erase(it--);
                }
            }
        }

        // Send handshake message to the other side every 0.5 second until we receive the ACK signal
        while(!end_handshake_topic_msg_){
            // Send handshake message every 0.5 seconds 
            std_msgs::String msg_to_send;
            msg_to_send.data = _goal_msg->bt_id + ":" + _goal_msg->message + "_1_ack";
            send_signal_publisher_.publish(msg_to_send);
            usleep(10e5);
        }
        std_msgs::Bool end_msg_to_send;
        end_msg_to_send.data = false;
        send_handshake_end_signal_publisher_.publish(end_msg_to_send);
    }

    void RosHandShake::ThreeWayHandshakeAustralia(const behavior_tree_ros::HandShakeGoalConstPtr& _goal_msg){
        bool first_sync_receive = false;
        bool second_sync_sent = false;

        while(!first_sync_receive){
            // Check if we have receive the first ACK signal from the other side
            if(new_handshake_topic_msg_){
                new_handshake_topic_msg_ = false;
                std::string handshake_bt_id {""};
                std::string handshake_message {""};
                for (auto it = handshake_topic_msgs_.begin(); it != handshake_topic_msgs_.end(); it++){
                    // ID and message are separated by ":"
                    size_t pos = it->find_first_of(":");
                    handshake_bt_id = it->substr(0, pos);
                    handshake_message = it->substr(pos+1, it->size());
                    // Checking if msg received is not empty, is from another BT and is in the same stage
                    if(!it->empty() && handshake_bt_id != _goal_msg->bt_id && handshake_message == _goal_msg->message+"_1")
                    {
                        ROS_INFO("Message received from another node [%s]", it->c_str());
                        // Remove matched message
                        handshake_topic_msgs_.erase(it--);
                        first_sync_receive = true;                                
                        break; // Stop checking msgs as we found what we were looking for
                    }
                    // Remove unmatched message
                    handshake_topic_msgs_.erase(it--);
                }
            }
        }

        while(!second_sync_sent){
            // Send handshake message every 0.5 seconds 
            std_msgs::String msg_to_send;
            msg_to_send.data = _goal_msg->bt_id + ":" + _goal_msg->message + "_ack";
            send_signal_publisher_.publish(msg_to_send);
            usleep(10e5);

            // Check if we have receive the ACK signal from the other side
            if(new_handshake_topic_msg_){
                new_handshake_topic_msg_ = false;
                std::string handshake_bt_id {""};
                std::string handshake_message {""};
                for (auto it = handshake_topic_msgs_.begin(); it != handshake_topic_msgs_.end(); it++){
                    // ID and message are separated by ":"
                    size_t pos = it->find_first_of(":");
                    handshake_bt_id = it->substr(0, pos);
                    handshake_message = it->substr(pos+1, it->size());
                    // Checking if msg received is not empty, is from another BT and is in the same stage
                    if(!it->empty() && handshake_bt_id != _goal_msg->bt_id && handshake_message == _goal_msg->message+"_1_ack")
                    {
                        ROS_INFO("Message received from another node [%s]", it->c_str());
                        // Remove matched message
                        handshake_topic_msgs_.erase(it--);
                        std_msgs::Bool end_msg_to_send;
                        end_msg_to_send.data = true;
                        send_handshake_end_signal_publisher_.publish(end_msg_to_send);
                        second_sync_sent = true;
                        end_handshake_topic_msg_ = true;                           
                        break; // Stop checking msgs as we found what we were looking for
                    }
                    // Remove unmatched message
                    handshake_topic_msgs_.erase(it--);
                }
            }
        }
    }

    void RosHandShake::HandShakeActionCallback(const behavior_tree_ros::HandShakeGoalConstPtr& _goal_msg)
    {
        ROS_INFO("Starting Handshake Action callback! [%s] [%s]", _goal_msg->bt_id.c_str(), _goal_msg->message.c_str());   
        std::string country_name;   
        ros::param::get("this_country", country_name);
        if(country_name == "japan"){
            ThreeWayHandshakeJapan(_goal_msg);
        }else{
            ThreeWayHandshakeAustralia(_goal_msg);
        }

        if(handshake_action_server_.isActive())
        {
            ROS_INFO("Handshake succeeded!!");
            end_handshake_topic_msg_ = false;
            handshake_action_result_.result = true;
            handshake_action_server_.setSucceeded(handshake_action_result_, "Synchronization succeeded!");
        }
    }    
    
    void RosHandShake::HandShakeActionPreemptCallback()
    {
        ROS_INFO("Handshake Action Goal canceled!");
        handshake_action_result_.result = false;
        handshake_action_server_.setPreempted(handshake_action_result_, "Goal preempted");
    }    
    
    RosExchangeInfo::RosExchangeInfo() :
        exchange_info_action_server_ (public_node_handle_, "behavior_tree/exchange_info", boost::bind(&RosExchangeInfo::ExchangeInfoActionCallback, this, _1), false)
    {
        // Get intercom topic name
        std::string exchange_info_topic_name = private_node_handle_.param<std::string>("exhange_info_topic_name", "/remote/bt_info");        // Publisher
        send_info_publisher_ = public_node_handle_.advertise<std_msgs::String>(exchange_info_topic_name, 1, true);        // Subscriber
        get_info_subscriber_ = public_node_handle_.subscribe(exchange_info_topic_name, 10, &RosExchangeInfo::ExchangeInfoTopicCallback, this);        // Actionlib
        exchange_info_action_server_.registerPreemptCallback(boost::bind(&RosExchangeInfo::ExchangeInfoActionPreemptCallback, this));
        exchange_info_action_server_.start();
    }    
    
    void RosExchangeInfo::ExchangeInfoTopicCallback(const std_msgs::StringConstPtr& _topic_msg)
    {
        ROS_INFO("Received Info topic message! [%s]", _topic_msg->data.c_str());
        std::unique_lock<std::mutex> lock (exchange_info_mutex_);
        exchange_info_topic_msgs_.emplace_back(_topic_msg->data);
        new_exchange_info_topic_msg_ = true;
    }    
    
    void RosExchangeInfo::ExchangeInfoActionCallback(const behavior_tree_ros::ExchangeInfoGoalConstPtr& _goal_msg)
    {
        ROS_INFO("Starting ExchangeInfo Action callback! [%s] [%s] [%s]", _goal_msg->bt_id.c_str(), _goal_msg->info_type.c_str(), _goal_msg->info_data.c_str());
        bool signal_sent = false;
        bool signal_received = false;        try
        {
            while(!signal_received && ros::ok() && exchange_info_action_server_.isActive())
            {
                // Check signal received is the correct one
                {
                    std::unique_lock<std::mutex> lock (exchange_info_mutex_);                    if(new_exchange_info_topic_msg_)
                    {
                        new_exchange_info_topic_msg_ = false;
                        std::string exchange_info_bt_id {""};
                        std::string exchange_info_type {""};
                        std::string exchange_info_data {""};
                        for (auto it = exchange_info_topic_msgs_.begin(); it != exchange_info_topic_msgs_.end(); it++)
                        {
                            // ID and message are separated by ":"
                            size_t pos = it->find_first_of(":");
                            size_t pos2 = it->find_last_of(":");
                            exchange_info_bt_id = it->substr(0, pos);
                            exchange_info_type = it->substr(pos+1, pos2-pos-1);
                            exchange_info_data = it->substr(pos2+1, it->size());                            ROS_INFO("EX Message received from another node [%s] [%s] [%s]", exchange_info_bt_id.c_str(), exchange_info_type.c_str(), exchange_info_data.c_str());
                            // Checking if msg received is not empty, is from another BT and is in the same stage
                            if(!it->empty() && exchange_info_bt_id != _goal_msg->bt_id && exchange_info_type == _goal_msg->info_type)
                            {
                                ROS_INFO("Message received from another node [%s]", it->c_str());
                                // Remove matched message
                                exchange_info_topic_msgs_.erase(it--);
                                signal_received = true;                                // Fill result message
                                exchange_info_action_result_.info_type = exchange_info_type;
                                exchange_info_action_result_.info_data = exchange_info_data;                                break; // Stop checking msgs as we found what we were looking for
                            }
                            // Remove unmatched message
                            exchange_info_topic_msgs_.erase(it--);
                        }
                    }
                }                // Send signal only one time
                if(!signal_sent)
                {
                    ROS_INFO("Sending signal to the other node!");
                    std_msgs::String msg_to_send;
                    msg_to_send.data = _goal_msg->bt_id + ":" + _goal_msg->info_type + ":" + _goal_msg->info_data;
                    send_info_publisher_.publish(msg_to_send);
                    signal_sent = true;
                }                usleep(2e5); // not overload CPU
            }            if(exchange_info_action_server_.isActive())
            {
                ROS_INFO("ExchangeInfo succeeded!!");
                exchange_info_action_result_.info_received = true;
                exchange_info_action_server_.setSucceeded(exchange_info_action_result_, "Synchronization succeeded!");
            }
        }
        catch(const std::runtime_error& ex)
        {
            ROS_ERROR("Error in ExchangeInfo Action: %s", ex.what());
            exchange_info_action_result_.info_received = false;
            exchange_info_action_server_.setAborted(exchange_info_action_result_, "Synchronization aborted!");
        }
    }    
    
    void RosExchangeInfo::ExchangeInfoActionPreemptCallback()
    {
        ROS_INFO("ExchangeInfo Action Goal canceled!");
        exchange_info_action_result_.info_received = false;
        exchange_info_action_server_.setPreempted(exchange_info_action_result_, "Goal preempted");
    }
}// namespace BT_ROS