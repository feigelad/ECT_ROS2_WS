/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
// #include <rclcpp/rclcpp.hpp>
#include "body_cmd_0x115.h"

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t BodyCmd115::Init(void *const msg)
      {
        /** Message Definition===============================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName          | @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       ADCU_LampCmd_Active    uint8_t      Motorola         7         1         1         0         0         1         /     ||
         *   2       ADCU_HeadLamp_Cmd      uint8_t      Motorola         15        1         1         0         0         1         /     ||
         *   3       ADCU_DblFlashLamp_Cmd  uint8_t      Motorola         14        1         1         0         0         1         /     ||
         *   4       ADCU_TurnLLamp_Cmd     uint8_t      Motorola         13        1         1         0         0         1         /     ||
         *   5       ADCU_TurnRLamp_Cmd     uint8_t      Motorola         12        1         1         0         0         1         /     ||
         *   6       ADCU_BackLamp_Cmd      uint8_t      Motorola         11        1         1         0         0         1         /     ||
         *   7       ADCU_Buzzer_Cmd        uint8_t      Motorola         10        1         1         0         0         1         /     ||
         *   8       ADCU_Horn_Cmd          uint8_t      Motorola         9         1         1         0         0         1         /     ||
         *   9       ADCU_RunLamp_Cmd       uint8_t      Motorola         8         1         1         0         0         1         /     ||
         *   10      ADCU_BrkLamp_Cmd       uint8_t      Motorola         23        1         1         0         0         1         /     ||
         *=================================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->cmd_msg.body_cmd_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_LampCmd_Active", MOTOROLA, 7, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_lampcmd_active.name,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_lampcmd_active.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_lampcmd_active.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_lampcmd_active.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_HeadLamp_Cmd", MOTOROLA, 15, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_headlamp_cmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_headlamp_cmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_headlamp_cmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_headlamp_cmd.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_DblFlashLamp_Cmd", MOTOROLA, 14, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_dblflashlamp_cmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_dblflashlamp_cmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_dblflashlamp_cmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_dblflashlamp_cmd.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_TurnLLamp_Cmd", MOTOROLA, 13, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_turnllamp_cmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_turnllamp_cmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_turnllamp_cmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_turnllamp_cmd.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_TurnRLamp_Cmd", MOTOROLA, 12, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_turnrlamp_cmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_turnrlamp_cmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_turnrlamp_cmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_turnrlamp_cmd.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_BackLamp_Cmd", MOTOROLA, 11, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_backlamp_cmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_backlamp_cmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_backlamp_cmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_backlamp_cmd.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Buzzer_Cmd", MOTOROLA, 10, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_buzzer_cmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_buzzer_cmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_buzzer_cmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_buzzer_cmd.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Horn_Cmd", MOTOROLA, 9, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_horn_cmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_horn_cmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_horn_cmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_horn_cmd.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_RunLamp_Cmd", MOTOROLA, 8, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_runlamp_cmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_runlamp_cmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_runlamp_cmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_runlamp_cmd.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_BrkLamp_Cmd", MOTOROLA, 23, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_brklamp_cmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_brklamp_cmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_brklamp_cmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.body_cmd_msg.adcu_brklamp_cmd.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void BodyCmd115::UpdateMsg(uint8_t *data, uint8_t len)
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
