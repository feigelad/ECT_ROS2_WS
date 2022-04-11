/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include "brake_cmd_0x111.h"

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t BrakeCmd111::Init(void *const msg)
      {
        /** Message Definition=========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName    |    @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       ADCU_Brk_Active     uint8_t      Motorola         7         1         1         0         0         1         /     ||
         *   2       ADCU_Brk_CtrlMode   uint8_t      Motorola         5         2         1         0         0         3         /     ||
         *   3       ADCU_Brk_TgtPedpos  float        Motorola         8         8        0.4        0         0        100        %     ||
         *   4       ADCU_Brk_TgtPress   float        Motorola        24        16        0.01       0         0        100       bar    ||
         *   5       ADCU_Brk_TgtAcc     float        Motorola        40        16        0.01      -20       -20       20        mps^2  ||
         *===========================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Brk_Active", MOTOROLA, 7, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_active.name,
                                                           ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_active.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_active.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_active.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Brk_CtrlMode", MOTOROLA, 5, 2, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_ctrlmode.name,
                                                           ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_ctrlmode.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_ctrlmode.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_ctrlmode.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("ADCU_Brk_TgtPedpos", MOTOROLA, 8, 8, 0.4, 0,
                                                       ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_tgtpedpos.name,
                                                       ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_tgtpedpos.unit,
                                                       &ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_tgtpedpos.time_stamp,
                                                       &ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_tgtpedpos.value, 0, 100, "%")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("ADCU_Brk_TgtPress", MOTOROLA, 24, 16, 0.01, 0,
                                                       ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_tgtpress.name,
                                                       ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_tgtpress.unit,
                                                       &ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_tgtpress.time_stamp,
                                                       &ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_tgtpress.value, 0, 100, "bar")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("ADCU_Brk_TgtAcc", MOTOROLA, 40, 16, 0.01, -20,
                                                       ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_tgtacc.name,
                                                       ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_tgtacc.unit,
                                                       &ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_tgtacc.time_stamp,
                                                       &ecar_chassis_ptr_->cmd_msg.brake_cmd_msg.adcu_brk_tgtacc.value, -20, 20, "mps^2")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void BrakeCmd111::UpdateMsg(uint8_t *data, uint8_t len)
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
