#pragma once

#include <atomic>
#include <thread>

#include "../../systime/systime.h"
#include "../../syslog/log_factory.h"

namespace nirvana
{
  namespace common
  {
    namespace schedule
    {
      class TaskThread
      {
      public:
        TaskThread()
        {
          is_inited_.exchange(false);
          is_started_.exchange(false);
          is_alive_.exchange(false);
          start_time_.exchange(0);
          last_run_time_.exchange(0);
          alive_time_.exchange(0);
        }
        virtual ~TaskThread()
        {
          is_started_.exchange(false);
          if (thread_.joinable())
            thread_.join();
        }
        bool Init(std::shared_ptr<common::time::ITime> timer, std::shared_ptr<common::log::ILog> logger, std::string name, std::function<void(void)> task_func, float std_period, float max_period_allowed, float alive_threshold);
        bool Start();
        bool Restart();
        bool Stop();
        bool CheckAlive();
        inline double StartTime() const
        {
          return start_time_.load();
        }
        inline double LastRunTime() const
        {
          return last_run_time_.load();
        }
        inline float AliveTime() const
        {
          return alive_time_.load();
        }

        inline float StandardPeriod() const
        {
          return std_period_.load();
        }

        inline float RealPeriod() const
        {
          return real_period_.load();
        }

        void ThreadFunc();

      private:
        std::shared_ptr<common::time::ITime> timer_;
        std::shared_ptr<common::log::ILog> logger_;
        std::atomic<bool> is_inited_;
        std::string name_;
        std::thread thread_;
        std::function<void(void)> task_func_;
        std::atomic<bool> is_started_;
        std::atomic<bool> is_alive_;
        std::atomic<float> std_period_;
        std::atomic<float> real_period_;
        std::atomic<float> max_period_allowed_;
        std::atomic<float> alive_threshold_;
        std::atomic<double> start_time_;
        std::atomic<double> last_run_time_;
        std::atomic<double> alive_time_;
      };
    }
  }
}