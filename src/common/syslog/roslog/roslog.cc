// #include "ros/ros.h"
// #include "rclcpp/rclcpp.hpp"
#include "roslog.h"
// #include "common/msg/quaternion.h"
#include <thread>

namespace nirvana
{
  namespace common
  {
    namespace log
    {
      RosLog::RosLog() 
      {
        is_inited_.exchange(false);
      }
      RosLog::~RosLog() {}

      bool RosLog::Init(void *node, std::shared_ptr<common::time::ITime> timer)
      {
        if(node == nullptr)
        {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Logger init error, node ptr is nullptr!"); //TODO:logger_name need to define, lifei, 2021/9/3
          is_inited_.exchange(false);
          return false;
        }
        node_ = (rclcpp::Node*)node;

        log_publisher_ = node_->create_publisher<::common::msg::LogData>("LogDataMsg", 10);
        if (timer == nullptr)
        {
          RCLCPP_ERROR(node_->get_logger(), "Logger init error !"); //TODO:logger_name need to define, lifei, 2021/9/3
          is_inited_.exchange(false);
          return false;
        }
        timer_ = timer;
        node_name_ = node_->get_name();
        is_inited_.exchange(true);
        return false;
      }

      void RosLog::Log(LogLevel lev, const std::string &str)
      {
        if (lev == Log_Debug)
          RCLCPP_DEBUG(node_->get_logger(), str.c_str());
        else if (lev == Log_Info)
          RCLCPP_INFO(node_->get_logger(), str.c_str());
        else if (lev == Log_Warn)
          RCLCPP_WARN(node_->get_logger(), str.c_str());
        else if (lev == Log_Error)
          RCLCPP_ERROR(node_->get_logger(), str.c_str());
        else if (lev == Log_Fatal)
          RCLCPP_FATAL(node_->get_logger(), str.c_str());
        else
        {
        }
        ::common::msg::LogData log_str;
        log_str.time_stamp = timer_->Now2Sec();
        log_str.log_type = static_cast<uint8_t>(lev);
        log_str.log_str = node_->get_name() + ':' + ' ' + str;
        log_publisher_->publish(log_str);
      }
      void RosLog::Log(LogLevel lev, const std::stringstream &strstr)
      {
        Log(lev, strstr.str());
      }
    } // namespace log
  }   // namespace common
} // namespace nirvana
