/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include "park_cmd_0x112.h"

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t ParkCmd112::Init(void *const msg)
      {
        /** Message Definition=========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName     | @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       ADCU_Prk_Active   uint8_t      Motorola         7         1         1         0         0         1         /     ||
         *   2       ADCU_Prk_Enable   uint8_t      Motorola         6         1         1         0         0         1         /     ||
         *===========================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->cmd_msg.park_cmd_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Prk_Active", MOTOROLA, 7, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.park_cmd_msg.adcu_prk_active.name,
                                                           ecar_chassis_ptr_->cmd_msg.park_cmd_msg.adcu_prk_active.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.park_cmd_msg.adcu_prk_active.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.park_cmd_msg.adcu_prk_active.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Prk_Enable", MOTOROLA, 6, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.park_cmd_msg.adcu_prk_enable.name,
                                                           ecar_chassis_ptr_->cmd_msg.park_cmd_msg.adcu_prk_enable.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.park_cmd_msg.adcu_prk_enable.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.park_cmd_msg.adcu_prk_enable.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void ParkCmd112::UpdateMsg(uint8_t *data, uint8_t len)
      {
        if (data == nullptr || is_inited_.load() == false)
        {
          return;
        }
        UpdateFrame(data, len);
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
