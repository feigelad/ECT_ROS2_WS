#include "ipc_message.h"

namespace nirvana
{
  namespace common
  {
    namespace communication
    {      
      std::string IpcMessage::Name()
      {
        std::unique_lock<std::mutex> lck(mutex_);
        return name_;
      }
      uint32_t IpcMessage::Period()
      {
        return 0;
      }
      MsgDirect IpcMessage::MsgDirection()
      {
        return msg_direct_;
      }
      int IpcMessage::SubMsgQueue(int queue_size, void *const msg)
      {
        if (MsgDirection() == MsgPub)
        {
          logger_->Log(common::log::Log_Warn, "Cannot sub msg, msg direction is publish !");
          return -1;
        }

        if (msg == nullptr)
        {
          logger_->Log(common::log::Log_Error, "Msg pointer is null !");
          return -1;
        }

        if (subscriber_ == nullptr)
        {
          logger_->Log(common::log::Log_Error, "Subscriber pointer is null, Init failed !");
          return -1;
        }

        return subscriber_->GetDataQueue(queue_size, msg);
      }
      bool IpcMessage::PubMsg(void *const msg)
      {
        if (MsgDirection() == MsgSub)
        {
          logger_->Log(common::log::Log_Warn, "Cannot pub msg, msg direction is subscribe !");
          return false;
        }

        if (msg == nullptr)
        {
          logger_->Log(common::log::Log_Error, "Msg pointer is null !");
          return false;
        }

        if (publisher_ == nullptr)
        {
          logger_->Log(common::log::Log_Error, "Publisher pointer is null, Init failed !");
          return false;
        }
        publisher_->Publish(msg);
        return true;
      }

    } // namespace communication
  }   // namespace common
} // namespace nirvana
