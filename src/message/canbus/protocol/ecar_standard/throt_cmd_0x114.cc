/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include "throt_cmd_0x114.h"

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t ThrotCmd114::Init(void *const msg)
      {
        /** Message Definition=========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName         | @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       ADCU_Drv_Active       uint8_t      Motorola         7         1         1         0         0         1         /     ||
         *   2       ADCU_Drv_CtrlMode     uint8_t      Motorola         5         2         1         0         0         3         /     ||
         *   3       ADCU_Drv_TgtGear      uint8_t      Motorola         0         2         1         0         0         3         /     ||
         *   4       ADCU_Drv_TgtPedpos    float        Motorola         8         8         1       -100      -100       100        %     ||
         *   5       ADCU_Drv_TgtVehSpd    float        Motorola         24        16       0.01     -100      -100       100      kmph    ||
         *   6       ADCU_Drv_TgtVehAccSpd float        Motorola         32        8        0.1       -15       -15        15      mps^2   ||
         *   7       ADCU_Drv_VehSpdLimit  float        Motorola         40        8        0.4        0         0        100      kmph    ||
         *===========================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Drv_Active", MOTOROLA, 7, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_active.name,
                                                           ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_active.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_active.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_active.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Drv_CtrlMode", MOTOROLA, 5, 2, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_ctrlmode.name,
                                                           ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_ctrlmode.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_ctrlmode.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_ctrlmode.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Drv_TgtGear", MOTOROLA, 0, 2, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_tgtgear.name,
                                                           ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_tgtgear.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_tgtgear.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_tgtgear.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("ADCU_Drv_TgtPedpos", MOTOROLA, 8, 8, 1, -100,
                                                       ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_tgtpedpos.name,
                                                       ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_tgtpedpos.unit,
                                                       &ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_tgtpedpos.time_stamp,
                                                       &ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_tgtpedpos.value, -100, 100, "%")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("ADCU_Drv_TgtVehSpd", MOTOROLA, 24, 16, 0.01, -100,
                                                       ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_tgtvehspd.name,
                                                       ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_tgtvehspd.unit,
                                                       &ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_tgtvehspd.time_stamp,
                                                       &ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_tgtvehspd.value, -100, 100, "kmph")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("ADCU_Drv_TgtVehAccSpd", MOTOROLA, 32, 8, 0.1, -15,
                                                       ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_tgtvehaccspd.name,
                                                       ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_tgtvehaccspd.unit,
                                                       &ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_tgtvehaccspd.time_stamp,
                                                       &ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_tgtvehaccspd.value, -15, 15, "mps^2")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("ADCU_Drv_VehSpdLimit", MOTOROLA, 40, 8, 0.4, 0,
                                                       ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_vehspdlimit.name,
                                                       ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_vehspdlimit.unit,
                                                       &ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_vehspdlimit.time_stamp,
                                                       &ecar_chassis_ptr_->cmd_msg.throt_cmd_msg.adcu_drv_vehspdlimit.value, 0, 100, "kmph")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void ThrotCmd114::UpdateMsg(uint8_t *data, uint8_t len)
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
