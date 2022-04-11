#pragma once
#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>
#include <map>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      typedef uint16_t canbus_error_t;

      const canbus_error_t CAN_RESULT_OK = 0;
      const canbus_error_t CAN_ERROR_BASE = CAN_RESULT_OK;
      const canbus_error_t CAN_ERROR_OPEN_DEVICE_FAILED = CAN_ERROR_BASE + 1;
      const canbus_error_t CAN_ERROR_INIT_DEVICE_FAILED = CAN_ERROR_BASE + 2;
      const canbus_error_t CAN_ERROR_START_FAILED = CAN_ERROR_BASE + 3;
      const canbus_error_t CAN_ERROR_SEND_FAILED = CAN_ERROR_BASE + 4;
      const canbus_error_t CAN_ERROR_RECV_FAILED = CAN_ERROR_BASE + 5;

      const canbus_error_t CAN_ERROR_MSGMANAGER_INIT_FAILED = CAN_ERROR_BASE + 6;
      const canbus_error_t CAN_ERROR_MSGMANAGER_START_FAILED = CAN_ERROR_BASE + 7;
      const canbus_error_t CAN_ERROR_MSGMANAGER_REGSIGNAL_FAILED = CAN_ERROR_BASE + 8;
      const canbus_error_t CAN_ERROR_MSGMANAGER_REGPROTOCOL_FAILED = CAN_ERROR_BASE + 9;

      const canbus_error_t CAN_ERROR_PROTOCOL_INIT_FAILED_BASE = 20;

#define CAN_ERROR_PROTOCOL_INIT_FAILED(x) (CAN_ERROR_PROTOCOL_INIT_FAILED_BASE + x)

      class CanBusError
      {
      public:
        static std::string GetCanbusErrorString(canbus_error_t err_stat)
        {
          std::map<canbus_error_t, std::string> can_err_map = {
              {CAN_RESULT_OK, "CAN Result OK."},
              {CAN_ERROR_OPEN_DEVICE_FAILED, "Can open device failed !"},
              {CAN_ERROR_INIT_DEVICE_FAILED, "Can init device failed !"},
              {CAN_ERROR_START_FAILED, "Can start failed !"},
              {CAN_ERROR_SEND_FAILED, "Can send failed !"},
              {CAN_ERROR_RECV_FAILED, "Can receive failed !"},
              {CAN_ERROR_MSGMANAGER_INIT_FAILED, "Can message manager init failed !"},
              {CAN_ERROR_MSGMANAGER_START_FAILED, "Can massage manager start failed !"},
              {CAN_ERROR_MSGMANAGER_REGSIGNAL_FAILED, "Can message manager register signal failed !"},
              {CAN_ERROR_MSGMANAGER_REGPROTOCOL_FAILED, "Can message manager register protocol failed !"}};

          auto itr = can_err_map.find(err_stat);
          if (itr == can_err_map.end())
            return "Undefined can error !";

          return can_err_map[err_stat];
        }
      };
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
