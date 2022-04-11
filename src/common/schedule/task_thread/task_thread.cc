#include "task_thread.h"

namespace nirvana
{
  namespace common
  {
    namespace schedule
    {
      bool TaskThread::Init(std::shared_ptr<common::time::ITime> timer, std::shared_ptr<common::log::ILog> logger, std::string name, std::function<void(void)> task_func, float std_period, float max_period_allowed, float alive_threshold)
      {
        if (timer == nullptr)
        {
          return false;
        }

        timer_ = timer;

        if (logger == nullptr)
        {
          return false;
        }
        logger_ = logger;

        if (name == "")
        {
          logger_->Log(nirvana::common::log::Log_Error, "Task thread init failed, the name is empty !");
          return false;
        }
        name_ = name;

        if (task_func == nullptr)
        {
          logger_->Log(nirvana::common::log::Log_Error, name_ + "thread init failed, the task function pointer is nullptr !");
          return false;
        }
        task_func_ = task_func;

        std_period_.exchange(std_period);

        if (max_period_allowed < std_period)
        {
          logger_->Log(nirvana::common::log::Log_Error, name_ + "thread init failed, the max period is lower than standard period !");
          return false;
        }
        max_period_allowed_.exchange(max_period_allowed);

        if (alive_threshold < max_period_allowed)
        {
          logger_->Log(nirvana::common::log::Log_Error, name_ + "thread init failed, the alive threshold is lower than max period !");
          return false;
        }
        alive_threshold_.exchange(alive_threshold);

        is_inited_.exchange(true);
        logger_->Log(nirvana::common::log::Log_Info, name + " thread init successful.");
        return true;
      }
      bool TaskThread::Start()
      {
        if (is_inited_.load() == false)
        {
          logger_->Log(nirvana::common::log::Log_Error, name_ + " thread start failed, the task thread is not inited !");
          return false;
        }

        if (is_started_.load() == true)
        {
          logger_->Log(nirvana::common::log::Log_Info, name_ + " thread is already started at time: " + std::to_string(start_time_.load()) + ".");
          return true;
        }

        logger_->Log(nirvana::common::log::Log_Info, name_ + " thread starting ...");
        is_started_.exchange(true);
        start_time_.exchange(timer_->Now2Sec());
        thread_ = std::thread(std::bind(&TaskThread::ThreadFunc, this));
        return true;
      }

      bool TaskThread::Restart()
      {
        Stop();
        Start();
      }
      bool TaskThread::Stop()
      {
        if (is_inited_.load() == false)
        {
          logger_->Log(nirvana::common::log::Log_Error, name_ + " thread stop failed, the task thread is not inited !");
          return false;
        }

        if (is_started_.load() == false)
        {
          logger_->Log(nirvana::common::log::Log_Info, name_ + " thread is already stoped at time: " + std::to_string(last_run_time_.load()) + ".");
          return true;
        }

        logger_->Log(nirvana::common::log::Log_Info, name_ + " thread stopping ...");
        alive_time_.exchange(timer_->Now2Sec() - start_time_.load());
        is_started_.exchange(false);
        if (thread_.joinable())
          thread_.join();
        return true;
      }

      bool TaskThread::CheckAlive()
      {
        double now = timer_->Now2Sec();
        if (now > last_run_time_.load() + alive_threshold_.load())
          is_alive_.exchange(false);
        else
          is_alive_.exchange(true);

        alive_time_.exchange(now - start_time_.load());
        return is_alive_.load();
      }

      void TaskThread::ThreadFunc()
      {
        logger_->Log(nirvana::common::log::Log_Info, name_ + " thread start successfull at time: " + std::to_string(start_time_.load()) + ".");
        int period_err_count = 0;
        while (is_started_.load())
        {
          double now = timer_->Now2Sec();
          float period = now - last_run_time_.load();
          if (period >= std::max<double>(0, std_period_.load() - 0.0025))
          {
            real_period_.exchange(period);
            alive_time_.exchange(now - start_time_.load());
            if (period > max_period_allowed_.load())
            {
              period_err_count++;
              if (period_err_count > 10)
              {
                logger_->Log(nirvana::common::log::Log_Warn, name_ + " thread is out of maximum period allowed , the period is up to " + std::to_string(period) + "s !");
                period_err_count = 0;
              }
            }
            else
              period_err_count = 0;
            last_run_time_.exchange(now);
            this->task_func_();
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        logger_->Log(nirvana::common::log::Log_Info, name_ + " thread is stoped at time: " + std::to_string(last_run_time_.load()) + ", alive time: " + std::to_string(alive_time_.load()) + " seconds.");
      }
    }
  }
}
