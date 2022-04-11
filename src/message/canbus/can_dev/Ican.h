/**
 * @file
 * @brief Defines the CanFrame struct and CanClient interface.
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>
#include <memory>

#include "../../../common/byte/byte.h"
#include "../can_comm/canbus_error.h"
#include "../../../common/syslog/log_factory.h"

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {

      const uint16_t MAX_CAN_SEND_BUFF_SIZE = 100;
      const uint16_t MAX_CAN_RECV_BUFF_SIZE = 100;

      /**
 * @struct CanFrmType
 * @brief The Struct which defines the CAN frame type
 */
      struct CanFrmType
      {
        uint32_t frm_id;
        uint8_t extend_flag;
        uint8_t frm_len;
        uint8_t frm_data[8];
        CanFrmType()
            : frm_id(0), extend_flag(0), frm_len(0)
        {
          std::memset(frm_data, 0, sizeof(frm_data));
        }

        std::string DebugString()
        {
          std::stringstream strm("");
          strm << "ID: 0x" << nirvana::common::Byte::byte_to_hex(frm_id, extend_flag)
               << ", Len: " << static_cast<int>(frm_len)
               << "Data: ";
          for (int i = 0; i < frm_len; i++)
          {
            strm << nirvana::common::Byte::byte_to_hex(frm_data[i]);
          }
          strm << ".";
          return strm.str();
        }
      };

      /**
 * @class CanApi
 * @brief The class which defines the CAN api to send and receive message.
 */
      class ICan
      {
      public:
        /**
   * @brief Constructor
   */
        ICan() = default;

        /**
   * @brief Destructor
   */
        virtual ~ICan() = default;

        /**
   * @brief Initialize the CAN client by specified CAN card parameters.
   * @param parameter CAN card parameters to initialize the CAN client.
   * @return If the initialization is successful.
   */
        virtual canbus_error_t Init(std::shared_ptr<common::log::ILog> logger, int dev_type, int dev_port, int dev_chan) = 0;

        /**
   * @brief Start the CAN client.
   * @return The status of the start action which is defined by
   *         apollo::common::ErrorCode.
   */
        virtual canbus_error_t Start() = 0;

        /**
   * @brief Stop the CAN client.
   */
        virtual void Stop() = 0;

        /**
   * @brief Send messages
   * @param frames The messages to send.
   * @param frame_num The amount of messages to send.
   * @return The status of the sending action which is defined by
   *         apollo::common::ErrorCode.
   */
        virtual canbus_error_t Send(const std::vector<CanFrmType> &frames,
                                    int32_t *const frame_num) = 0;

        /**
   * @brief Receive messages
   * @param frames The messages to receive.
   * @param frame_num The amount of messages to receive.
   * @return The status of the receiving action which is defined by
   *         apollo::common::ErrorCode.
   */
        virtual canbus_error_t Receive(std::vector<CanFrmType> *const frames,
                                       int32_t *const frame_num) = 0;

        /**
   * @brief Get the error string.
   * @param status The status to get the error string.
   */
        virtual std::string GetErrorString(const int32_t status) = 0;
      };

    } // namespace canbus
  }   // namespace message
} // namespace nirvana
