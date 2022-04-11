#include "canbus_message_manager.h"

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {

      canbus_error_t CanbusMessageManager::Init(std::shared_ptr<common::log::ILog> logger,
                                                std::shared_ptr<common::time::ITime> timer,
                                                std::shared_ptr<ICan> can_api,
                                                bool enable_recv_log,
                                                bool enable_send_log)
      {
        if (logger == nullptr)
        {
          is_inited_.exchange(false);
          return CAN_ERROR_MSGMANAGER_INIT_FAILED;
        }

        logger_ = logger;

        if (timer == nullptr)
        {
          is_inited_.exchange(false);
          return CAN_ERROR_MSGMANAGER_INIT_FAILED;
        }
        timer_ = timer;

        if (can_api == nullptr)
        {
          logger_->Log(common::log::Log_Error, "Invalid can api instance.");
          is_inited_.exchange(false);
          return CAN_ERROR_MSGMANAGER_INIT_FAILED;
        }

        // Can api Instance
        can_api_ = can_api;
        enable_recv_log_.exchange(enable_recv_log);
        enable_send_log_.exchange(enable_send_log);

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }
      canbus_error_t CanbusMessageManager::Start()
      {
        if (is_inited_.load() == false)
        {
          logger_->Log(common::log::Log_Error, "CanbusMessage is not inited !");
          is_started_.exchange(false);
          return CAN_ERROR_MSGMANAGER_INIT_FAILED;
        }

        if (can_api_->Start() != CAN_RESULT_OK)
        {
          logger_->Log(common::log::Log_Error, "Failed to start can api instance.");
          return CAN_ERROR_MSGMANAGER_START_FAILED;
        }
        logger_->Log(common::log::Log_Info, "Can api instance is started.");

        is_started_.exchange(true);
        return CAN_RESULT_OK;
      }

      void CanbusMessageManager::Stop()
      {
        if (IsRunning())
        {
          logger_->Log(common::log::Log_Info, "Stop CanbusMessage !");
          is_started_.exchange(false);
          logger_->Log(common::log::Log_Info, "CanbusMessage is stopped !");
        }

        can_api_->Stop();
        logger_->Log(common::log::Log_Info, "CanbusMessage is not running !");
      }

      bool CanbusMessageManager::IsRunning() const
      {
        return is_started_.load();
      }

      canbus_error_t CanbusMessageManager::RegistProtocol(const std::shared_ptr<CanbusProtocol> &protocol, void *const msg)
      {
        auto itr = canbus_protocol_map_.find(protocol->ID());
        if (itr != canbus_protocol_map_.end())
        {
          //protocol already exists
          return CAN_ERROR_MSGMANAGER_REGPROTOCOL_FAILED;
        }
        canbus_protocol_map_[protocol->ID()] = protocol;

        return canbus_protocol_map_[protocol->ID()]->Init(msg);
      }

      //TODO: How to get real period of received frams，LiFei, 2020/7/6
      canbus_error_t CanbusMessageManager::CanbusRecvMsgFunc(void *const msg)
      {
        if (IsRunning())
        {
          int32_t recv_len = 0;
          std::vector<CanFrmType> recv_buff;
          recv_buff.clear();
          canbus_error_t err = can_api_->Receive(&recv_buff, &recv_len);
          if (err != CAN_RESULT_OK)
          {
            //TODO: How to handle recv frame failed, LiFei, 2020/7/6
            return err;
          }

          for (int i = 0; i < recv_buff.size(); i++)
          {
            std::unique_lock<std::mutex> lck(mutex_);
            uint32_t uid = recv_buff[i].frm_id;
            auto itr = canbus_protocol_map_.find(uid);
            if (itr == canbus_protocol_map_.end() || canbus_protocol_map_[uid]->Direct() != MSG_RECV)
              continue;

            canbus_protocol_map_[uid]->ParseMsg(recv_buff[i].frm_data, recv_buff[i].frm_len, timer_->Now2Sec());
            canbus_protocol_map_[uid]->SetStamp(timer_->Now2Sec());
            if (enable_recv_log_.load())
            {
              //TODO: How to log received can frames, LiFei, 2020/7/6
            }
          }

          return CAN_RESULT_OK;
        }
      }

      //TODO: How to get real period of send frams，LiFei, 2020/7/6
      canbus_error_t CanbusMessageManager::CanbusSendMsgFunc(void *const msg)
      {
        if (IsRunning())
        {
          std::vector<CanFrmType> send_frames;
          send_frames.clear();
          {
            std::unique_lock<std::mutex> lck(mutex_);
            for (auto itr = canbus_protocol_map_.begin(); itr != canbus_protocol_map_.end(); itr++)
            {
              if (itr->second->Direct() == MSG_SEND && timer_->Now2Sec() - itr->second->LastFrmStamp() >= (itr->second->Period() * 0.001 - 0.005))
              {
                CanFrmType send_frm;
                send_frm.frm_id = itr->second->ID();
                // std::stringstream strstr;
                // static double last_stamp = timer_->Now2Sec();
                // static double period = 0;
                // if (send_frm.frm_id == 0x111)
                // {
                //   period = timer_->Now2Sec() - last_stamp;
                //   last_stamp = timer_->Now2Sec();
                // }
                // strstr << itr->second->ID() << ": " << std::setprecision(20) << timer_->Now2Sec() << " [" << period << "].";
                // logger_->Log(common::log::Log_Info, strstr);
                send_frm.extend_flag = itr->second->IdExtend();
                send_frm.frm_len = 8;
                itr->second->UpdateMsg(send_frm.frm_data, send_frm.frm_len);
                send_frames.push_back(send_frm);
                itr->second->SetStamp(timer_->Now2Sec());
              }
            }
          }
          int send_len = send_frames.size();
          if (send_len > 0)
            return can_api_->Send(send_frames, &send_len);
          else
            return CAN_RESULT_OK;
        }
      }

    } // namespace canbus
  }   // namespace message
} // namespace nirvana
