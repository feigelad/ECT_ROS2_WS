#pragma once

#include <string>
#include <unordered_map>

#include "../ipc_message/ipc_message.h"

namespace nirvana
{
  namespace common
  {
    namespace communication
    {
      class IpcManager
      {
      public:
        IpcManager()
          :is_ipc_thread_started(false) {}
        virtual ~IpcManager()
        {
          message_map_.clear();
        }

        bool Init(void *node, const std::shared_ptr<log::ILog> logger);
        template <typename MsgType>
        bool RegistMsg(std::string name, communication::MsgDirect direct);
        bool PubMsg(std::string name, void *const msg);
        int SubMsg(std::string name, int buff_size, void *const msg);
        void Start();

      private:
        std::thread ipc_thread_;
        bool is_ipc_thread_started;
        void IpcTask();

      private:
        rclcpp::Node *node_;
        std::shared_ptr<log::ILog> logger_;
        std::unordered_map<std::string, std::shared_ptr<IpcMessage>> message_map_;
        std::mutex mutex_;
        // std::string node_name_;
      };

      template <typename MsgType>
      bool IpcManager::RegistMsg(std::string name, communication::MsgDirect direct)
      {
        if (message_map_.find(name) != message_map_.end())
        {
          if (direct != message_map_[name]->MsgDirection())
          {
            logger_->Log(log::Log_Error, name + " exsisits, but direction is not match !");
            return false;
          }
          logger_->Log(log::Log_Info, name + " exsisits !");
          return true;
        }
        message_map_[name] = std::shared_ptr<IpcMessage>(new IpcMessage());
        return message_map_[name]->Init<MsgType>(node_, name, logger_, direct);
      }
    } // namespace communication
  }   // namespace common
} // namespace nirvana
