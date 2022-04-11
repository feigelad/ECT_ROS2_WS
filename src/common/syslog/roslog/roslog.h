#pragma once

// #include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"
#include "../ILog.h"
#include <mutex>
#include <atomic>

#include "common/msg/log_data.hpp"

namespace nirvana
{
  namespace common
  {
    namespace log
    {
      class RosLog : public ILog
      {
      public:
        RosLog();
        virtual ~RosLog();
        virtual bool Init(void *node, std::shared_ptr<common::time::ITime> timer) override;
        virtual void Log(LogLevel lev, const std::string &str) override;
        void Log(LogLevel lev, const std::stringstream &strstr) override;

      private:
        // ros::NodeHandle nh_;
        // ros::Publisher log_publisher_;
        rclcpp::Node *node_;
        rclcpp::Publisher<::common::msg::LogData>::SharedPtr log_publisher_;
        std::shared_ptr<common::time::ITime> timer_;

        std::atomic<bool> is_inited_;
        mutable std::mutex mutex_;
        std::string node_name_;
      };
    } // namespace log
  }   // namespace common
} // namespace nirvana
