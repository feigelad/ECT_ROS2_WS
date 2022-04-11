#pragma once

#include <unordered_map>
#include <thread>

#include "../../systime/systime.h"
#include "../../syslog/log_factory.h"
#include "../../communication/ipc_manager/ipc_manager.h"
#include "../task_thread/task_thread.h"

namespace nirvana
{
  namespace common
  {
    namespace schedule
    {
      class ThreadManager
      {
      public:
        ThreadManager()
        {
          is_inited_.exchange(false);
        }
        virtual ~ThreadManager()
        {
          guard_thread_->Stop();
        }
        bool Init(std::string node_name, std::shared_ptr<common::time::ITime> timer, std::shared_ptr<common::log::ILog> logger, std::shared_ptr<common::communication::IpcManager> ipc_manager);
        bool RegistTaskThread(std::string name, std::function<void(void)> task_func, float std_period, float max_period_allowed, float alive_threshold);
        bool Start(std::string name);
        bool Stop(std::string name);
        bool Restart(std::string name);
        bool StartAll();
        bool StopAll();
        bool RestartAll();

      private:
        void GuardFunc();

      private:
        std::shared_ptr<common::time::ITime> timer_;
        std::shared_ptr<common::log::ILog> logger_;
        std::shared_ptr<common::communication::IpcManager> ipc_manager_;
        std::atomic<bool> is_inited_;
        mutable std::mutex mutex_;
        std::string node_name_;
        std::unordered_map<std::string, std::shared_ptr<TaskThread>> thread_map_;
        std::shared_ptr<TaskThread> guard_thread_;
      };
    }
  }
}