/*
 * @Descripttion:
 * @version:
 * @Author: LiFei
 * @Date: 2020-02-06 11:05:39
 * @LastEditors  : LiFei
 * @LastEditTime : 2020-02-14 23:06:39
 */

#include <iostream>
#include <string>
#include <unordered_map>
#include <unistd.h>
#include <cstdlib>

#include "../../../../common/byte/byte.h"
#include "zlg_can.h"
// #include "ros/ros.h"

using namespace nirvana::common;

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      namespace zlg_can
      {
        // ZlgCan::CanApi(){}

        ZlgCan::~ZlgCan()
        {
          // Stop();
          CloseDevice(dev_type_, 0);
        }

        canbus_error_t ZlgCan::Init(std::shared_ptr<common::log::ILog> logger, int dev_type, int dev_port, int dev_chan)
        {
          logger_ = logger;
          dev_type_ = dev_type;
          dev_port_ = dev_port;
          dev_chan_ = dev_chan;
          return CAN_RESULT_OK;
        }

        canbus_error_t ZlgCan::Start()
        {
          if (is_started_)
          {
            return CAN_RESULT_OK;
          }
          CloseDevice(dev_type_, 0);

          // uint8_t extern_flag = 0;
          // if (FLAGS_esd_can_extended_frame) {
          //   extern_flag = 1;
          // }

          /*Open device*/
          if (STATUS_OK != VCI_OpenDevice(dev_type_, 0, 0))
          {
            is_started_ = false;
            //Log
            std::stringstream log_strstr;
            log_strstr << "CAN open failed, device type: [" << dev_type_ << "].";
            logger_->Log(common::log::Log_Error, log_strstr.str());

            return CAN_ERROR_OPEN_DEVICE_FAILED;
          }
          /*Set reference*/
          if ((dev_type_ == VCI_USBCAN_E_U) ||
              (dev_type_ == VCI_USBCAN_2E_U))
          {
            DWORD baud = 0x060007;
            if (STATUS_OK != VCI_SetReference(dev_type_, 0,
                                              dev_port_, 0,
                                              &baud))
            {
              CloseDevice(dev_type_, 0);
              return CAN_ERROR_OPEN_DEVICE_FAILED;
            }
          }
          /*Init device*/
          VCI_INIT_CONFIG vic;
          vic.AccCode = 0x0;
          vic.AccMask = 0xffffffff;
          vic.Filter = 1;
          vic.Mode = 0;
          vic.Timing0 = 0x00;
          vic.Timing1 = 0x1c; // 500kbps
          if (STATUS_OK != VCI_InitCAN(dev_type_, 0,
                                       dev_port_, &vic))
          {
            //Log
            std::stringstream log_strstr;
            log_strstr << "CAN init failed, device type: [" << dev_type_ << "].";
            logger_->Log(common::log::Log_Error, log_strstr.str());

            CloseDevice(dev_type_, 0);
            is_started_ = false;
            return CAN_ERROR_INIT_DEVICE_FAILED;
          }
          /*Start device*/
          if (STATUS_OK != VCI_StartCAN(dev_type_, 0,
                                        dev_port_))
          {
            //Log
            std::stringstream log_strstr;
            log_strstr << "CAN start failed, device type: [" << dev_type_ << "].";
            logger_->Log(common::log::Log_Error, log_strstr.str());

            CloseDevice(dev_type_, 0);
            is_started_ = false;
            return CAN_ERROR_START_FAILED;
          }

          is_started_ = true;
          return CAN_RESULT_OK;
        }

        void ZlgCan::Stop()
        {
          if (is_started_)
          {
            is_started_ = false;
            CloseDevice(dev_type_, 0);
          }
        }

        canbus_error_t ZlgCan::Send(const std::vector<CanFrmType> &frames,
                                    int32_t *const frame_num)
        {
          if (frames.empty())
          {
            // ROS_DEBUG_STREAM()
            return CAN_RESULT_OK;
          }

          if (false == is_started_)
          {
            //Log
            std::stringstream log_strstr;
            log_strstr << "CAN has not been started, device type: [" << dev_type_ << "].";
            logger_->Log(common::log::Log_Error, log_strstr.str());

            return CAN_ERROR_START_FAILED;
          }
          size_t send_len = std::min<size_t>(
              frames.size(), static_cast<size_t>(MAX_CAN_SEND_BUFF_SIZE));
          for (size_t i = 0; i < send_len; ++i)
          {
            send_frames_[i].ID = frames[i].frm_id;
            send_frames_[i].RemoteFlag = 0;
            send_frames_[i].ExternFlag = frames[i].extend_flag ? 1 : 0;
            send_frames_[i].SendType = 0;
            send_frames_[i].DataLen = frames[i].frm_len;
            std::memcpy(send_frames_[i].Data, &frames[i].frm_data, frames[i].frm_len);
          }

          if (send_len != VCI_Transmit(dev_type_, 0, dev_port_, send_frames_, send_len))
          {
            VCI_ERR_INFO err_info;
            if (STATUS_OK == VCI_ReadErrInfo(dev_type_, 0, dev_port_, &err_info))
            {
              //Log
              std::stringstream log_strstr;
              log_strstr << "Can send message failed, error code is: [" << err_info.ErrCode << "]/{" << GetErrorString(static_cast<int32_t>(err_info.ErrCode)) << "}";
              logger_->Log(common::log::Log_Error, log_strstr.str());
            }
            else
            {
              //Log
              std::stringstream log_strstr;
              log_strstr << "CAN send message failed, device type: [" << dev_type_ << "].";
              logger_->Log(common::log::Log_Error, log_strstr.str());
            }
            return CAN_ERROR_SEND_FAILED;
          }

          // ROS_INFO_STREAM("ID:" << send_frames_[0].ID);

          return CAN_RESULT_OK;
        }

        canbus_error_t ZlgCan::Receive(std::vector<CanFrmType> *const frames,
                                       int32_t *const frame_num)
        {
          if (false == is_started_)
          {
            // ROS_ERROR("CAN receive failed: Zlg can is not inited, please init first !");
            return CAN_ERROR_START_FAILED;
          }
          *frame_num = 0;
          // int len = VCI_GetReceiveNum(dev_type_, 0, dev_port_);
          int recved_len = VCI_Receive(dev_type_, 0, dev_port_, recv_frames_, 100, 10);
          VCI_ClearBuffer(dev_type_, 0, dev_port_);

          // std::cout << "recv " << recved_len << " can messages. ";

          if (recved_len > 0)
          {
            frames->clear();
            for (int i = 0; i < recved_len; i++)
            {
              CanFrmType frm;
              frm.frm_id = recv_frames_[i].ID;
              frm.extend_flag = recv_frames_[i].ExternFlag;
              frm.frm_len = recv_frames_[i].DataLen;
              std::memcpy(frm.frm_data, recv_frames_[i].Data, frm.frm_len);
              frames->push_back(frm);
            }
            *frame_num = recved_len;
            return CAN_RESULT_OK;
          }
          else if (recved_len == -1)
          {
            VCI_ERR_INFO err_info;
            if (STATUS_OK == VCI_ReadErrInfo(dev_type_, 0, dev_port_, &err_info))
            {
              //Log
              std::stringstream log_strstr;
              log_strstr << "Can recv message failed, error code is " << err_info.ErrCode << ", " << GetErrorString(err_info.ErrCode);
              logger_->Log(common::log::Log_Error, log_strstr);
            }
            else
            {
              logger_->Log(common::log::Log_Error, "Can recv message failed !");
            }
            return CAN_ERROR_RECV_FAILED;
          }
          return CAN_RESULT_OK;
        }
        std::string ZlgCan::GetErrorString(const int32_t status)
        {
          std::unordered_map<uint32_t, std::string> err_map{
              {ERR_CAN_OVERFLOW, "ERR_CAN_OVERFLOW"},
              {ERR_CAN_ERRALARM, "ERR_CAN_ERRALARM"},
              {ERR_CAN_PASSIVE, "ERR_CAN_PASSIVE"},
              {ERR_CAN_LOSE, "ERR_CAN_LOSE"},
              {ERR_CAN_BUSERR, "ERR_CAN_BUSERR"},
#ifdef ERR_CAN_BUSOFF
              {ERR_CAN_BUSOFF, "ERR_CAN_BUSOFF"},
#endif
              {ERR_DEVICEOPENED, "ERR_DEVICEOPENED"},
              {ERR_DEVICEOPEN, "ERR_DEVICEOPEN"},
              {ERR_DEVICENOTOPEN, "ERR_DEVICENOTOPEN"},
              {ERR_BUFFEROVERFLOW, "ERR_BUFFEROVERFLOW"},
              {ERR_DEVICENOTEXIST, "ERR_DEVICENOTEXIST"},
              {ERR_LOADKERNELDLL, "ERR_LOADKERNELDLL"},
              {ERR_CMDFAILED, "ERR_CMDFAILED"},
              {ERR_BUFFERCREATE, "ERR_BUFFERCREATE"}};
          auto iter = err_map.find(status);
          if (iter == err_map.end())
          {
            return "No ErrorCode !";
          }
          return err_map[status];
        }

        void ZlgCan::CloseDevice(uint32_t dev_type, uint32_t dev_ind)
        {
          if (!is_started_)
            VCI_OpenDevice(dev_type, dev_ind, 0);
          usleep(50000);
          VCI_ResetCAN(dev_type, dev_ind, 0);
          VCI_CloseDevice(dev_type, dev_ind);
          usleep(50000);
        }

      } // namespace zlg_can
    }   // namespace canbus
  }     // namespace message
} // namespace nirvana