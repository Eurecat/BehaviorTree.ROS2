#pragma once

#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h> 

#include <cstdint>
#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <algorithm>
#include <numeric>

#include "rclcpp/rclcpp.hpp"

/*#include "ros/ros.h"
#include "ros/console.h"
#include "rospack/rospack.h"*/

class ROS2LaunchManager 
{
    std::vector<pid_t> m_pids;

    std::atomic<bool> m_running;
    std::thread m_thread;
    std::mutex m_mutex;
    
public:
    ROS2LaunchManager(ROS2LaunchManager const &)
    {
    }

    ROS2LaunchManager()
    {
        std::atomic_init(&m_running, true);

        m_thread = std::thread(&ROS2LaunchManager::wait, this);
    }

    ~ROS2LaunchManager() 
    {
        if (m_running.load()) {
            m_running.store(false);

            if (m_thread.joinable()) {
                m_thread.join();
            }
        }
    }

    template<typename... Args>
    pid_t start(const rclcpp::Node::SharedPtr& nh, Args... args) 
    {
        std::vector<std::string> args_vector = { args... };

        if (args_vector.size() > 0) {
            pid_t pid = ::fork();
           // int r = prctl(PR_SET_PDEATHSIG, SIGTERM);
            if (pid == 0) {
                RCLCPP_INFO(nh->get_logger(),"PID EQUALS TO 0");
                ::setsid();
                
                ::signal(SIGINT, SIG_IGN);
                
                /*::fclose(stdout);
                ::fclose(stdin);
                ::fclose(stderr);*/

                ::execlp("ros2", "ros2", "run", args..., nullptr);
            }
            else {
                RCLCPP_INFO(nh->get_logger(),"PID NOT EQUALS TO 0");
                std::scoped_lock<std::mutex> scoped_lock(m_mutex);

                std::string args_string = std::accumulate(std::next(std::begin(args_vector)), std::end(args_vector), args_vector[0], [](std::string lhs, std::string rhs) -> std::string { return lhs + " " + rhs; });

                RCLCPP_INFO(nh->get_logger(),"Starting \"ros2 %s\" with PID %d", args_string.c_str(), pid);

                m_pids.push_back(pid);
            }

            return pid;
        }
        else {
            throw std::runtime_error("ROSLaunchManager::start - No arguments provided");
        }
    }

    void stop(const rclcpp::Node::SharedPtr& nh, pid_t const &pid, int32_t const &signal) 
    {
        std::scoped_lock<std::mutex> scoped_lock(m_mutex);

        auto pid_it = std::find(std::begin(m_pids), std::end(m_pids), pid);

        if (pid_it != m_pids.end()) {
            ::kill(pid, signal);

            RCLCPP_INFO(nh->get_logger(),"Stopping process with PID %d and signal %d", pid, signal);
        }
        else {
            throw std::runtime_error("ROSLaunchManager::stop - PID " + std::to_string(pid) + " not found");
        }
    }

private:
    void wait()
    {
        while (m_running.load()) {
            std::scoped_lock<std::mutex> scoped_lock(m_mutex);

            for (auto pid_it = std::begin(m_pids); pid_it != std::end(m_pids); ++pid_it) {
                pid_t const pid = *pid_it;

                int32_t status;

                if (::waitpid(pid, &status, WUNTRACED | WCONTINUED | WNOHANG) == pid) {
                    if (WIFEXITED(status)) {
                      //  RCLCPP_INFO(nh->get_logger(),"PID %d exited with status %d", pid, WEXITSTATUS(status));

                        pid_it = m_pids.erase(pid_it);

                        if (pid_it == std::end(m_pids)) {
                            break;
                        }
                    } 
                    else if (WIFSIGNALED(status)) {
                     //   RCLCPP_INFO(nh->get_logger(),"PID %d killed with signal %d", pid, WTERMSIG(status));

                        pid_it = m_pids.erase(pid_it);

                        if (pid_it == std::end(m_pids)) {
                            break;
                        }
                    } 
                    else if (WIFSTOPPED(status)) {
                      //  RCLCPP_INFO(nh->get_logger(),"PID %d stopped with signal %d", pid, WSTOPSIG(status));
                    } 
                    else if (WIFCONTINUED(status)) {
                      //  RCLCPP_INFO(nh->get_logger(),"PID %d continued"   , pid);
                    }
                }
            }
        }

        std::scoped_lock<std::mutex> scoped_lock(m_mutex);

        for (pid_t const &pid : m_pids) {
            ::kill(pid, SIGINT);

            int32_t status;

            ::waitpid(pid, &status, 0);
        }
    }
};