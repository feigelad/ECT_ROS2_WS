/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-16 16:01:10
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 01:17:48
 */

#pragma once
#include <mutex>
#include "../canbus_protocol.h"
#include "message/msg/ecar_chassis.hpp"

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      /** Protocols Definition=======================================================
      *  LiFei, 2020/7/7
      *  @SN |   @MsgId  | @MsgName              | @Ide | @Period(ms) | @Send/Recv||
      *   1       0x213    CDCU_ParkStatus           0      20             Recv   ||
      *==========================================================================**/

      class ParkStat213 : public CanbusProtocol
      {
      public:
        ParkStat213()
            : CanbusProtocol(0x213, "CDCU_ParkStatus", 0, 20, MSG_RECV)
        {
        }

        virtual canbus_error_t Init(void *const msg) override;

        virtual void ParseMsg(const uint8_t *data, uint8_t len, double stamp);

      private:
        std::atomic<bool> is_inited_;
        ::message::msg::EcarChassis *ecar_chassis_ptr_;
      };
    } // namespace canbus
  }   // namespace message
} // namespace nirvana