/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include "power_cmd_0x117.h"

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t PowerCmd117::Init(void *const msg)
      {
        /** Message Definition===============================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName            | @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       ADCU_VehPwrup_Cmd        uint8_t      Motorola         6         2         1         0         0         3         /     ||
         *   2       ADCU_ChasPwrup_Cmd       uint8_t      Motorola         4         2         1         0         0         3         /     ||
         *   3       ADCU_12V1Pwrup_Cmd       uint8_t      Motorola         2         2         1         0         0         3         /     ||
         *   4       ADCU_12V2Pwrup_Cmd       uint8_t      Motorola         0         2         1         0         0         3         /     ||
         *   5       ADCU_AutDrv24VPwrup_Cmd  uint8_t      Motorola         14        2         1         0         0         3         /     ||
         *=================================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->cmd_msg.power_cmd_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_VehPwrup_Cmd", MOTOROLA, 6, 2, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_vehpwrup_cmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_vehpwrup_cmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_vehpwrup_cmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_vehpwrup_cmd.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_ChasPwrup_Cmd", MOTOROLA, 4, 2, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_chaspwrup_cmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_chaspwrup_cmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_chaspwrup_cmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_chaspwrup_cmd.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_12V1Pwrup_Cmd", MOTOROLA, 2, 2, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_12v1pwrup_cmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_12v1pwrup_cmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_12v1pwrup_cmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_12v1pwrup_cmd.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_12V2Pwrup_Cmd", MOTOROLA, 0, 2, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_12v2pwrup_cmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_12v2pwrup_cmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_12v2pwrup_cmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_12v2pwrup_cmd.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_AutDrv24VPwrup_Cmd", MOTOROLA, 14, 2, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_autdrv24vpwrup_cmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_autdrv24vpwrup_cmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_autdrv24vpwrup_cmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.power_cmd_msg.adcu_autdrv24vpwrup_cmd.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void PowerCmd117::UpdateMsg(uint8_t *data, uint8_t len)
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
