#pragma once

#include <atomic>
#include <future>
#include <memory>
#include <unordered_map>
#include <thread>

#include "../can_dev/Ican.h"
#include "../protocol/canbus_protocol.h"
#include "../../../common/systime/systime.h"

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {

      class CanbusMessageManager
      {
      public:
        CanbusMessageManager(/* args */) {}
        virtual ~CanbusMessageManager()
        {
        }

        virtual canbus_error_t Init(std::shared_ptr<common::log::ILog> logger,
                                    std::shared_ptr<common::time::ITime> timer,
                                    std::shared_ptr<ICan> can_api,
                                    bool enable_recv_log,
                                    bool enable_send_log);
        virtual canbus_error_t Start();
        virtual void Stop();
        bool IsRunning() const;
        template <typename SignalType>
        canbus_error_t RegistSignal(std::string msg_name, uint32_t msg_id, uint8_t msg_ide, uint32_t msg_prd,
                                    enum MessageDirect direct, std::string sig_name, ByteOrder sig_order,
                                    uint8_t sig_pos, uint8_t sig_len, float factor, float offset,
                                    SignalType *const val, SignalType max_val, SignalType min_val, std::string unit)
        {
          if (val == nullptr)
          {
            return CAN_ERROR_MSGMANAGER_REGSIGNAL_FAILED;
          }

          auto itr = canbus_protocol_map_.find(msg_id);
          if (itr == canbus_protocol_map_.end())
          {
            canbus_protocol_map_[msg_id] = std::shared_ptr<CanbusProtocol>(new CanbusProtocol(msg_id, msg_name, msg_ide, msg_prd, direct));
          }

          if (canbus_protocol_map_[msg_id]->Name() != msg_name || canbus_protocol_map_[msg_id]->IdExtend() != msg_ide || canbus_protocol_map_[msg_id]->Period() != msg_prd || canbus_protocol_map_[msg_id]->Direct() != direct)
          {
            return CAN_ERROR_MSGMANAGER_REGSIGNAL_FAILED;
          }

          if (false == canbus_protocol_map_[msg_id]->AddSignal(CanSignal<SignalType>(sig_name, sig_order,
                                                                                     sig_pos, sig_len, factor,
                                                                                     offset, val, max_val, min_val, unit)))
          {
            return CAN_ERROR_MSGMANAGER_REGSIGNAL_FAILED;
          }
          return CAN_RESULT_OK;
        }
        canbus_error_t RegistProtocol(const std::shared_ptr<CanbusProtocol> &protocol, void *const msg);

      public:
        canbus_error_t CanbusRecvMsgFunc(void *const msg);
        canbus_error_t CanbusSendMsgFunc(void *const msg);

      private:
        std::atomic<bool> is_inited_;
        std::atomic<bool> is_started_;
        std::atomic<bool> enable_recv_log_;
        std::atomic<bool> enable_send_log_;
        std::shared_ptr<common::log::ILog> logger_;
        std::shared_ptr<common::time::ITime> timer_;
        std::shared_ptr<ICan> can_api_;

        mutable std::mutex mutex_;
        std::unordered_map<uint32_t, std::shared_ptr<CanbusProtocol>> canbus_protocol_map_;
      };
    } // namespace canbus
  }   // namespace message
} // namespace nirvana