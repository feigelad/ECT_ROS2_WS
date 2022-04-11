#include "ipc_manager.h"

namespace nirvana
{
  namespace common
  {
    namespace communication
    {
      bool IpcManager::Init(void *node, const std::shared_ptr<log::ILog> logger)
      {
        if (node == nullptr)
          return false;
        node_ = (rclcpp::Node*)node;

        message_map_.clear();
        if (logger == nullptr)
          return false;

        logger_ = logger;
        return true;
      }

      bool IpcManager::PubMsg(std::string name, void *const msg)
      {
        if (message_map_.find(name) == message_map_.end())
        {
          logger_->Log(log::Log_Error, "Message is not registered !");
          return false;
        }

        if (message_map_[name]->MsgDirection() != MsgPub)
        {
          logger_->Log(log::Log_Error, "Message direction does not match !");
          return false;
        }

        return message_map_[name]->PubMsg(msg);
      }
      int IpcManager::SubMsg(std::string name, int buff_size, void *const msg)
      {
        static int s_msg_empty_counter = 0;
        if (message_map_.find(name) == message_map_.end())
        {
          logger_->Log(log::Log_Error, "Message is not registered !");
          return -1;
        }

        if (message_map_[name]->MsgDirection() != MsgSub)
        {
          logger_->Log(log::Log_Error, "Message direction does not match !");
          return -1;
        }

        int len = message_map_[name]->SubMsgQueue(buff_size, msg);
        if (len < 0)
        {
          s_msg_empty_counter++;
          if (s_msg_empty_counter > 10)
          {
            logger_->Log(log::Log_Warn, "IpcManager can not subscribe valid " + name + " !");
            s_msg_empty_counter = 0;
          }
        }

        return len;
      }

      void IpcManager::Start()
      {
        is_ipc_thread_started = true;
        ipc_thread_ = std::thread(&IpcManager::IpcTask, this);
      }

      //privates
      void IpcManager::IpcTask()
      {        
        rclcpp::spin(std::shared_ptr<rclcpp::Node>(node_));
      }

    } // namespace communication
  }   // namespace common
} // namespace nirvana
