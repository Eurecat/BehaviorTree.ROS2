#include "BehaviorTreeIntercomNode.hpp"

#define SYNC_MSG "SYNC"
#define SYNC_ACK_MSG "SYNC_ACK"
#define FINAL_ACK_MSG "ACK"

namespace BT_ROS
{
    using namespace behavior_tree_ros;

    RosHandShake::RosHandShake() :
        handshake_action_server_ (public_node_handle_, "behavior_tree/handshake", boost::bind(&RosHandShake::ThreeWayHandShakeActionCallback, this, _1), false)
    {

        // Get intercom topic names
        std::string handshake_server_topic_name = private_node_handle_.param<std::string>("handshake_server_topic_name", "/server/bt_handshake");  
        std::string handshake_client_topic_name = private_node_handle_.param<std::string>("handshake_client_topic_name", "/client/bt_handshake"); 

        ros::param::get("/handshake_mode", handshake_mode_);

        if (handshake_mode_ == "server")
        {
            // Publisher of updates on server side
            sync_publisher_ = public_node_handle_.advertise<HandShake>(handshake_server_topic_name, 1, true);    

            //Subscriber to updates from client side
            sync_subscriber_ =  public_node_handle_.subscribe(handshake_client_topic_name, 10, &RosHandShake::ThreeWayHandShakeTopicCallbackServer, this);    
        }
        else
        {       
            // Publisher of updates on client side
            sync_publisher_ = public_node_handle_.advertise<HandShake>(handshake_client_topic_name, 1, true);    
                                          
            //Subscriber of updates from server side
            sync_subscriber_ =  public_node_handle_.subscribe(handshake_server_topic_name, 10, &RosHandShake::ThreeWayHandShakeTopicCallbackClient, this);    
        }

        // Actionlib
        handshake_action_server_.registerPreemptCallback(boost::bind(&RosHandShake::ThreeWayHandShakeActionPreemptCallback, this));
        handshake_action_server_.start();

        // Publish Timer
        pub_timer_ = public_node_handle_.createTimer(ros::Duration(pub_period_s_), &RosHandShake::PublishSmsCallback, this, false, false);
    }    
    
    void RosHandShake::PublishSmsCallback(const ros::TimerEvent& ev)
    {
        sync_publisher_.publish(msg_to_send_);
        std::string mode_str = "CLIENT";
        if (handshake_mode_ == "server")
        {
            mode_str = "SERVER";
        }
        ROS_INFO("%s: %s Message send %d", mode_str.c_str(), msg_to_send_.message.c_str(), msg_to_send_.seq_id);
    }

    void RosHandShake::ThreeWayHandShakeActionCallback(const PerformHandShakeGoalConstPtr& _goal_msg)
    {
        action_cancelled_.store(false);
        my_seq_id_ = _goal_msg->request.seq_id;
        data_to_send = _goal_msg->request.data;
        if (handshake_mode_ == "server"){
            ThreeWayHandshakeServer(_goal_msg);
        }else{
            ThreeWayHandshakeClient(_goal_msg);
        }

        if(handshake_action_server_.isActive())
        {
            ROS_INFO("Handshake succeeded!!"); 

            handshake_result_.result = true;
	        handshake_result_.data = data_rx;
            handshake_action_server_.setSucceeded(handshake_result_, "Synchronization succeeded!");
        }
            }    

    void RosHandShake::ThreeWayHandshakeServer(const PerformHandShakeGoalConstPtr& _goal_msg){
        
        ROS_INFO("ThreeWayHandshake Server START");

        sync_received_.store(false);
        ack_received_.store(false);

        // Wait for SYNC handshake message from the other side every 2 second
        ROS_INFO("SERVER: Waiting FOR SYNC message %d [%d-%s]", my_seq_id_, _goal_msg->request.seq_id, _goal_msg->request.message.c_str());
        bool cancelled = action_cancelled_.load();
        while(!sync_received_.load() && !cancelled)
        {
            usleep(100000);
            cancelled = action_cancelled_.load();
        }
        if (cancelled) return;

        //Send SYNC_ACK periodically
        msg_to_send_.message = SYNC_ACK_MSG;
        msg_to_send_.seq_id = my_seq_id_;
        msg_to_send_.data = data_to_send;
        sync_publisher_.publish(msg_to_send_);
        ROS_INFO("SERVER: %s Message send %d", msg_to_send_.message.c_str(), msg_to_send_.seq_id);
        pub_timer_.start();

        // Wait for ACK handshake message from the other side, while pub SYNC_ACK
        ROS_INFO("SERVER: Waiting FOR ACK message %d [%d-%s]", my_seq_id_, _goal_msg->request.seq_id, _goal_msg->request.message.c_str());
        while(!ack_received_.load() && !action_cancelled_.load())
        {          
            usleep(100000);
        }
        //Stop sending SYNC_ACK
        pub_timer_.stop();
    }

    void RosHandShake::ThreeWayHandshakeClient(const PerformHandShakeGoalConstPtr& _goal_msg){
        
        ROS_INFO("ThreeWayHandshake Client START");

        ack_received_.store(false);

        //Send SYNC periodically
        msg_to_send_.message = SYNC_MSG;
        msg_to_send_.seq_id = my_seq_id_;
        msg_to_send_.data = data_to_send;
        sync_publisher_.publish(msg_to_send_);
        ROS_INFO("CLIENT: %s Message send %d", msg_to_send_.message.c_str(), msg_to_send_.seq_id);
        pub_timer_.start();
        ROS_INFO("CLIENT: Waiting FOR SYNC_ACK message %d [%d-%s]", my_seq_id_, _goal_msg->request.seq_id, _goal_msg->request.message.c_str());
        // Wait for SYNC_ACK signal
        while(!ack_received_.load() && !action_cancelled_.load())
        {            
            usleep(100000);
        }
        //Stop sending SYNC
        pub_timer_.stop();
    }
    
    void RosHandShake::ThreeWayHandShakeActionPreemptCallback()
    {
        ROS_INFO("Handshake Action Goal canceled!");

        my_seq_id_ = -1;
        data_to_send = "";
        action_cancelled_.store(true);
        sync_received_.store(false);
        ack_received_.store(false);
        
        handshake_result_.result = false;
        handshake_result_.data = "";
        handshake_action_server_.setPreempted(handshake_result_, "Goal preempted");
    }    

    void RosHandShake::ThreeWayHandShakeTopicCallbackClient(const behavior_tree_ros::HandShake& _topic_msg)
    {
        if(my_seq_id_ == _topic_msg.seq_id)
        {
            // NORMAL CASE
            if(_topic_msg.message == SYNC_ACK_MSG)
            {
                data_rx = _topic_msg.data;
                ROS_INFO("CLIENT: Received SYNC_ACK message %d [%d-%s]", my_seq_id_, _topic_msg.seq_id, _topic_msg.message.c_str());
                HandShake msg_to_send;
                msg_to_send.message = FINAL_ACK_MSG;
                msg_to_send.seq_id = my_seq_id_;
                msg_to_send.data = "";
                ROS_INFO("CLIENT: Send ACK message %d [%d-%s]", my_seq_id_, my_seq_id_, _topic_msg.message.c_str());
                sync_publisher_.publish(msg_to_send);
                
                ack_received_.store(true); // RECEIVED SYNC_ACK of stage we're both in
            }
        }   
        else if((my_seq_id_ > _topic_msg.seq_id) && (_topic_msg.seq_id != -1))
        {
            // CLIENT IS AT STAGE N+x, COMMUNICATE FINAL ACK FOR STAGE N, SO THAT SERVER CAN REACH US
            if(_topic_msg.message == SYNC_ACK_MSG)
            {
		        data_rx = _topic_msg.data;
                HandShake msg_to_send;
                msg_to_send.message = FINAL_ACK_MSG;
                msg_to_send.seq_id = _topic_msg.seq_id;
                msg_to_send.data = "";
                ROS_INFO("CLIENT: Received PREV SYNC_ACK message %d [%d-%s] & Replying ACK Message", my_seq_id_, _topic_msg.seq_id, _topic_msg.message.c_str());
                sync_publisher_.publish(msg_to_send);
            }
        }
    }

    void RosHandShake::ThreeWayHandShakeTopicCallbackServer(const behavior_tree_ros::HandShake& _topic_msg)
    {

        if(my_seq_id_ == _topic_msg.seq_id)
        {
            // NORMAL CASE
            if(_topic_msg.message == SYNC_MSG)
            {
                ROS_INFO("SERVER: Received SYNC message %d [%d-%s]", my_seq_id_, _topic_msg.seq_id, _topic_msg.message.c_str());
                data_rx = _topic_msg.data;
                sync_received_.store(true); // RECEIVED SYNC of stage we're both in
            }
            
            else if(_topic_msg.message == FINAL_ACK_MSG)
            {
                ROS_INFO("SERVER: Received ACK message %d [%d-%s]", my_seq_id_, _topic_msg.seq_id, _topic_msg.message.c_str());
                ack_received_.store(true); // RECEIVED FINAL ACK of stage we're both in
            }
        }  
        else if((my_seq_id_ > _topic_msg.seq_id) && (_topic_msg.seq_id != -1))
        {
            // SERVER IS AT STAGE N+x, COMMUNICATE SYNC ACK FOR STAGE N, SO THAT CLIENT CAN REACH US
            if(_topic_msg.message == SYNC_MSG)
            {
                data_rx = _topic_msg.data;
                ROS_INFO("SERVER: Received PREV SYNC message %d [%d-%s] & Replying SYNC-ACK Message", my_seq_id_, _topic_msg.seq_id, _topic_msg.message.c_str());
                HandShake msg_to_send;
                msg_to_send.message = SYNC_ACK_MSG;
                msg_to_send.seq_id = _topic_msg.seq_id;
                msg_to_send.data = data_to_send;
                sync_publisher_.publish(msg_to_send);
            }
        }
        else if((my_seq_id_ < _topic_msg.seq_id) && (my_seq_id_ != -1))
        {
            // SERVER IS AT STAGE N-x, COMMUNICATE SYNC ACK FOR STAGE N-x, SO THAT CLIENT CAN SEND FINAL ACK AND SERVER CAN MOVE FORWARD
            if(_topic_msg.message == SYNC_MSG)
            {
                data_rx = _topic_msg.data;
                ROS_INFO("SERVER: Received NEXT SYNC message %d [%d-%s] & Replying SYNC-ACK Message from previous sync", my_seq_id_, _topic_msg.seq_id, _topic_msg.message.c_str());
                HandShake msg_to_send;
                msg_to_send.message = SYNC_ACK_MSG;
                msg_to_send.seq_id = my_seq_id_;
                msg_to_send.data = data_to_send;
                sync_publisher_.publish(msg_to_send);
                sync_received_.store(true);
            }
        }
    }
    
    RosExchangeInfo::RosExchangeInfo() :
        exchange_info_action_server_ (public_node_handle_, "behavior_tree/exchange_info", boost::bind(&RosExchangeInfo::ExchangeInfoActionCallback, this, _1), false)
    {
        // Get intercom topic name
        std::string exchange_info_topic_name = private_node_handle_.param<std::string>("exhange_info_topic_name", "/remote/bt_info");               // Publisher
        send_info_publisher_ = public_node_handle_.advertise<std_msgs::String>(exchange_info_topic_name, 1, true);                                  // Subscriber
        get_info_subscriber_ = public_node_handle_.subscribe(exchange_info_topic_name, 10, &RosExchangeInfo::ExchangeInfoTopicCallback, this);      // Actionlib
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