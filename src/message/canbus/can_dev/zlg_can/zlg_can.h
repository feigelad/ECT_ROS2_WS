/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-06 11:05:26
 * @LastEditors  : LiFei
 * @LastEditTime : 2020-02-14 23:34:08
 */
#pragma once

#include <string>
#include <vector>
#include "../Ican.h"
#include "../../third_party/zlg_can/controlcan.h"

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      namespace zlg_can
      {

        /**
         * @brief Zlg Can Card Class inherited from CanClient
         */
        class ZlgCan
            : public nirvana::message::canbus::ICan
        {
        public:
          ZlgCan() {}
          /**
           * @brief Destructor
           */
          virtual ~ZlgCan();

          /**
            * @brief Initialize the Example CAN client by specified CAN card parameters.
            * @param parameter CAN card parameters to initialize the CAN client.
            * @return If the initialization is successful.
            */
          canbus_error_t Init(std::shared_ptr<common::log::ILog> logger, int dev_type, int dev_port, int dev_chan) override;
          /**
            * @brief Start the Example CAN client.
            * @return The status of the start action which is defined by
            *         apollo::common::ErrorCode.
            */
          canbus_error_t Start() override;

          /**
            * @brief Stop the Example CAN client.
            */
          void Stop() override;

          /**
            * @brief Send messages
            * @param frames The messages to send.
            * @param frame_num The amount of messages to send.
            * @return The status of the sending action which is defined by
            *         apollo::common::ErrorCode.
            */
          canbus_error_t Send(const std::vector<CanFrmType> &frames,
                              int32_t *const frame_num) override;

          /**
            * @brief Receive messages
            * @param frames The messages to receive.
            * @param frame_num The amount of messages to receive.
            * @return The status of the receiving action which is defined by
            *         apollo::common::ErrorCode.
            */
          canbus_error_t Receive(std::vector<CanFrmType> *const frames,
                                 int32_t *const frame_num) override;

          /**
            * @brief Get the error string.
            * @param status The status to get the error string.
            */
          std::string GetErrorString(const int32_t status) override;

        private:
          void CloseDevice(uint32_t dev_type, uint32_t dev_ind);

        private:
          /// The CAN client is started.
          bool is_started_ = false;
          std::shared_ptr<common::log::ILog> logger_;
          int dev_type_;
          int dev_port_;
          int dev_chan_;
          int baud_rate_;
          VCI_CAN_OBJ send_frames_[MAX_CAN_SEND_BUFF_SIZE];
          VCI_CAN_OBJ recv_frames_[MAX_CAN_RECV_BUFF_SIZE];
        };

      } // namespace zlg_can
    }   // namespace canbus
  }     // namespace message
} // namespace nirvana
