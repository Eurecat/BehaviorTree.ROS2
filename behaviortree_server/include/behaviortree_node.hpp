
#ifndef BEHAVIORTREE_NODE_H
#define BEHAVIORTREE_NODE_H

#include "rclcpp/rclcpp.hpp"

#include "tree_wrapper.hpp"
#include "std_srvs/srv/empty.hpp"
#include "behaviortree_server_interfaces/msg/bb_entry.hpp"
#include "behaviortree_server_interfaces/msg/transition.hpp"
#include "behaviortree_server_interfaces/srv/get_loaded_plugins.hpp"
#include "behaviortree_server_interfaces/srv/get_tree_status.hpp"
#include "behaviortree_server_interfaces/srv/get_bb_values.hpp"

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include "3rdparty/tinyxml2/tinyxml2.h"


using std::placeholders::_1;
using std::placeholders::_2;

using BBEntry = behaviortree_server_interfaces::msg::BBEntry;
using GetLoadedPluginsSrv = behaviortree_server_interfaces::srv::GetLoadedPlugins;
using GetTreeStatusSrv = behaviortree_server_interfaces::srv::GetTreeStatus;
using GetBBValues = behaviortree_server_interfaces::srv::GetBBValues;
using TreeStatus = behaviortree_server_interfaces::msg::TreeExecutionStatus;
using Transition = behaviortree_server_interfaces::msg::Transition;

using EmptySrv = std_srvs::srv::Empty;

class BehaviorTreeNode
{
  private:
    
  public:
    BehaviorTreeNode(const rclcpp::Node::SharedPtr& node);

    void getParameters (rclcpp::Node::SharedPtr nh);
    bool GetLoadedPluginsService(const std::shared_ptr<GetLoadedPluginsSrv::Request> _request, std::shared_ptr<GetLoadedPluginsSrv::Response> _response);
    bool StopTree(const std::shared_ptr<EmptySrv::Request> _request, std::shared_ptr<EmptySrv::Response> _response);
    bool RestartTree(const std::shared_ptr<EmptySrv::Request> _request, std::shared_ptr<EmptySrv::Response> _response);
    bool StatusTree(const std::shared_ptr<GetTreeStatusSrv::Request> _request, std::shared_ptr<GetTreeStatusSrv::Response> _response);
    void SyncBlackboardUpdateCallback(const BBEntry::SharedPtr _topic_msg);
    void LoadAllPlugins();
    void LoadPluginsFromROS();
    void LoadPluginsFromFolder(const std::string& _folder);
    void LoadPlugin(const std::string& _plugin_path);
    void InitializeBlackboard(const std::string& abs_file_path, BT::Blackboard::Ptr blackboard_ptr, const bool sync_bb);
    void InitializeStatusPublisher(std::string tree_name);
    bool AreLoggersInitialized();
    void PublishExecutionStatus(bool error=false, std::string error_data="");
    void InitializeLoggers();
    
    std::string GetFullPath(const std::string& _file) const;
    void Loop();

    rclcpp::Node::SharedPtr node_ ;
    BT::TreeWrapper tree_wrapper_;
    std::string trees_folder_;
    std::string tree_name_;
    std::string tree_filename_;
    std::vector<std::string> tree_bb_init_ {};
    int tree_uid_;
    bool tree_debug_;
    bool tree_auto_restart_;
    int tree_server_port_;
    int tree_publisher_port_;
    std::string log_folder_;
    bool enable_cout_log_;
    bool enable_minitrace_log_;
    bool enable_rostopic_log_;
    bool enable_file_log_;
    bool enable_zmq_log_;
    bool loggers_init_ = false;

    rclcpp::Time start_execution_time_;
    std::set<std::string> loaded_plugins_;

    //Services
    rclcpp::Service<GetLoadedPluginsSrv>::SharedPtr get_loaded_plugins_srv_;
    rclcpp::Service<EmptySrv>::SharedPtr stop_tree_srv_;
    rclcpp::Service<EmptySrv>::SharedPtr restart_tree_srv_;
    rclcpp::Service<GetTreeStatusSrv>::SharedPtr get_tree_status_srv_;

    //Subscribers
    rclcpp::Subscription<BBEntry>::SharedPtr sync_bb_sub_;

    //Publishers
    rclcpp::Publisher<BBEntry>::SharedPtr sync_bb_pub_;
    rclcpp::Publisher<Transition>::SharedPtr bt_transition_publisher_;
    rclcpp::Publisher<TreeStatus>::SharedPtr bt_execution_status_publisher_;

    std::shared_ptr<BT::Groot2Publisher> groot_publisher_;
};



#endif  // BEHAVIORTREE_NODE_H