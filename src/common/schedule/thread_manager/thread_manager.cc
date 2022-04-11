#include "thread_manager.h"
#include "common/msg/thread_manager.hpp"
namespace nirvana
{
  namespace common
  {
    namespace schedule
    {
      bool ThreadManager::Init(std::string node_name, std::shared_ptr<common::time::ITime> timer, std::shared_ptr<common::log::ILog> logger, std::shared_ptr<common::communication::IpcManager> ipc_manager)
      {
        if (timer == nullptr)
          return false;
        timer_ = timer;

        if (logger == nullptr)
          return false;
        logger_ = logger;

        if (node_name == "")
        {
          logger_->Log(nirvana::common::log::Log_Error, "Thread manager inited failed, node name is empty !");
          return false;
        }
        node_name_ = node_name;

        if (ipc_manager == nullptr)
        {
          logger_->Log(nirvana::common::log::Log_Error, " thread manager inited failed, ipc_manager is nullptr !");
          return false;
        }
        ipc_manager_ = ipc_manager;

        //Regist ipc_message
        if (false == ipc_manager_->RegistMsg<::common::msg::ThreadManager>("ScheduleMsg", nirvana::common::communication::MsgPub))
        {
          logger_->Log(nirvana::common::log::Log_Error, " thread manager inited failed, ipc_manager regist ScheduleMsg failed !");
          return false;
        }
        logger_->Log(nirvana::common::log::Log_Info, "ipc_manager regist ScheduleMsg successfull.");

        guard_thread_ = std::shared_ptr<common::schedule::TaskThread>(new common::schedule::TaskThread());
        if (false == guard_thread_->Init(timer_, logger_, "thread_manager_guard", std::bind(&ThreadManager::GuardFunc, this), 0.1, 0.3, 0.5))
        {
          logger_->Log(nirvana::common::log::Log_Error, " thread manager inited failed, guard thread inited failed !");
          return false;
        }
        if (false == guard_thread_->Start())
        {
          logger_->Log(nirvana::common::log::Log_Error, " thread manager inited failed, guard thread started failed !");
          return false;
        }

        logger_->Log(nirvana::common::log::Log_Info, " thread manager inited successfull.");
        is_inited_.exchange(true);
        return true;
      }
      bool ThreadManager::RegistTaskThread(std::string name, std::function<void(void)> task_func, float std_period, float max_period_allowed, float alive_threshold)
      {
        if (is_inited_.load() == false)
        {
          logger_->Log(nirvana::common::log::Log_Error, " thread manager regist task failed, thread manager is not inited !");
          return false;
        }

        if (name == "")
        {
          logger_->Log(nirvana::common::log::Log_Error, " thread manager regist task failed, task name is empty !");
          return false;
        }

        if (task_func == nullptr)
        {
          logger_->Log(nirvana::common::log::Log_Error, " thread manager regist task failed, task function is nullptr !");
          return false;
        }

        if (std_period < 0 || max_period_allowed < std_period || alive_threshold < max_period_allowed)
        {
          logger_->Log(nirvana::common::log::Log_Error, " thread manager regist task failed, task period params is fault !");
          return false;
        }

        std::unique_lock<std::mutex> lck(mutex_);
        if (thread_map_.find(name) != thread_map_.end())
        {
          logger_->Log(nirvana::common::log::Log_Error, " thread manager regist task failed, task[" + name + "] exists !");
          return false;
        }

        thread_map_[name] = std::shared_ptr<common::schedule::TaskThread>(new common::schedule::TaskThread());
        if (thread_map_[name]->Init(timer_, logger_, name, task_func, std_period, max_period_allowed, alive_threshold) == false)
        {
          thread_map_.erase(name);
          logger_->Log(nirvana::common::log::Log_Error, " thread manager regist task failed, task[" + name + "] inited failed !");
          return false;
        }

        logger_->Log(nirvana::common::log::Log_Info, " thread manager regist task[" + name + "] successfull.");
        return true;
      }
      bool ThreadManager::Start(std::string name)
      {
        std::unique_lock<std::mutex> lck(mutex_);
        auto itr = thread_map_.find(name);
        if (itr == thread_map_.end())
        {
          logger_->Log(nirvana::common::log::Log_Error, " thread manager start task failed, task[" + name + "] does not exist !");
          return false;
        }

        if (itr->second->Start() == false)
        {
          logger_->Log(nirvana::common::log::Log_Error, " thread manager start task [" + itr->first + "] failed !");
          return false;
        }

        logger_->Log(nirvana::common::log::Log_Info, " thread manager start task [" + itr->first + "] successfull.");
        return true;
      }
      bool ThreadManager::Stop(std::string name)
      {
        std::unique_lock<std::mutex> lck(mutex_);
        auto itr = thread_map_.find(name);
        if (itr == thread_map_.end())
        {
          logger_->Log(nirvana::common::log::Log_Error, " thread manager stop task failed, task[" + name + "] does not exist !");
          return false;
        }

        if (itr->second->Stop() == false)
        {
          logger_->Log(nirvana::common::log::Log_Error, " thread manager stop task [" + itr->first + "] failed !");
          return false;
        }

        logger_->Log(nirvana::common::log::Log_Info, " thread manager stop task [" + itr->first + "] successfull.");
        return true;
      }
      bool ThreadManager::Restart(std::string name)
      {
      }
      bool ThreadManager::StartAll()
      {
        bool start_flag = true;
        std::unique_lock<std::mutex> lck(mutex_);
        if (thread_map_.empty())
        {
          logger_->Log(nirvana::common::log::Log_Error, " thread manager has no task to start !");
          return false;
        }

        for (auto itr = thread_map_.begin(); itr != thread_map_.end(); itr++)
        {
          if (itr->second->Start() == false)
          {
            logger_->Log(nirvana::common::log::Log_Error, " thread manager start task [" + itr->first + "] failed !");
            start_flag = false;
          }
        }

        if (start_flag == false)
        {
          logger_->Log(nirvana::common::log::Log_Error, " thread manager start some task failed !");
          return false;
        }

        logger_->Log(nirvana::common::log::Log_Info, " thread manager start all task successfull.");
        return true;
      }

      bool ThreadManager::StopAll()
      {
        bool stop_flag = true;
        std::unique_lock<std::mutex> lck(mutex_);
        if (thread_map_.empty())
        {
          logger_->Log(nirvana::common::log::Log_Error, " thread manager has no task to stop !");
          return false;
        }

        for (auto itr = thread_map_.begin(); itr != thread_map_.end(); itr++)
        {
          if (itr->second->Stop() == false)
          {
            logger_->Log(nirvana::common::log::Log_Error, " thread manager stop task [" + itr->first + "] failed !");
            stop_flag = false;
          }
        }

        if (stop_flag == false)
        {
          logger_->Log(nirvana::common::log::Log_Error, " thread manager stop some task failed !");
          return false;
        }

        logger_->Log(nirvana::common::log::Log_Info, " thread manager stop all task successfull.");
        return true;
      }
      bool ThreadManager::RestartAll()
      {
      }

      void ThreadManager::GuardFunc()
      {
        std::unordered_map<std::string, std::shared_ptr<TaskThread>> thread_map;
        std::string node_name;
        {
          std::unique_lock<std::mutex> lck(mutex_);
          thread_map = thread_map_;
          node_name = node_name_;
        }

        for (auto itr = thread_map.begin(); itr != thread_map.end(); itr++)
        {
          ::common::msg::ThreadManager thread_manager_msg;
          thread_manager_msg.header.frame_id = "ScheduleMsg";
          rclcpp::Time now;
          thread_manager_msg.header.stamp = now;
          thread_manager_msg.node_name = node_name;
          thread_manager_msg.task_name = itr->first;
          thread_manager_msg.is_alive = itr->second->CheckAlive();
          thread_manager_msg.start_time = itr->second->StartTime();
          thread_manager_msg.last_run_time = itr->second->LastRunTime();
          thread_manager_msg.alive_time = itr->second->AliveTime();
          thread_manager_msg.standard_period = itr->second->StandardPeriod();
          thread_manager_msg.real_period = itr->second->RealPeriod();

          ipc_manager_->PubMsg("ScheduleMsg", &thread_manager_msg);
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      }
    }
  }
}