#pragma once

#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <typeinfo>
#include <memory>

#include "../../syslog/log_factory.h"
#include "../../communication/publisher/publisher_factory.h"
#include "../../communication/subscriber/subscriber_factory.h"

namespace nirvana
{
  namespace common
  {
    namespace communication
    {
      enum MsgDirect
      {
        MsgPub = 0,
        MsgSub
      };

      class IpcMessage
      {
      public:
        IpcMessage() {}
        virtual ~IpcMessage() {}
        template <typename MsgType>
        bool Init(void* node, std::string name, const std::shared_ptr<common::log::ILog> &logger, const common::communication::MsgDirect direct);
        std::string Name();
        uint32_t Period();
        MsgDirect MsgDirection();
        int SubMsgQueue(int queue_size, void *const msg);
        bool PubMsg(void *const msg);

      protected:
        mutable std::mutex mutex_;
        std::string name_;
        std::shared_ptr<log::ILog> logger_;
        std::shared_ptr<communication::IPublisher> publisher_;
        std::shared_ptr<communication::ISubscriber> subscriber_;
        common::communication::MsgDirect msg_direct_;
      };

      template <typename MsgType>
      bool IpcMessage::Init(void* node, std::string name, const std::shared_ptr<common::log::ILog> &logger, const common::communication::MsgDirect direct)
      {
        if (node == nullptr)
        {
          logger_->Log(common::log::Log_Error, "ipc message node is empty !");
          return false;
        }

        if (name == "")
        {
          logger_->Log(common::log::Log_Error, "ipc message name is empty !");
          return false;
        }
        name_ = name;

        if (logger == nullptr)
          return false;

        logger_ = logger;

        msg_direct_ = direct;

        if (direct == common::communication::MsgPub)
        {
          common::communication::PublisherFactory pub_factory;
          publisher_ = std::shared_ptr<common::communication::IPublisher>(pub_factory.CreateRosPublisher<MsgType>(node, Name(), 10)); //TODO:buff_size or qos_history_depth
          subscriber_ = nullptr;
          if (publisher_ == nullptr)
          {
            logger_->Log(common::log::Log_Error, name_ + " publisher created failed !");
            return false;
          }
        }
        else
        {
          common::communication::SubscriberFactory sub_factory;
          subscriber_ = std::shared_ptr<common::communication::ISubscriber>(sub_factory.CreateRosSubscriber<MsgType>(node, Name(),10)); //TODO:buff_size or qos_history_depth
          publisher_ = nullptr;
          if (subscriber_ == nullptr)
          {
            logger_->Log(common::log::Log_Error, name_ + " subscriber created failed !");
            return false;
          }
        }
        logger_->Log(common::log::Log_Info, name_ + " init successfully !");
        return true;
      }

    } // namespace communication
  }   // namespace common
} // namespace nirvana
