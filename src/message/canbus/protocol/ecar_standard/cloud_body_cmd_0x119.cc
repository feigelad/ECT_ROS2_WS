/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include "cloud_body_cmd_0x119.h"

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t CloudBodyCmd119::Init(void *const msg)
      {
        /** Message Definition===============================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName             | @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       ADCU_Cld_HeadLampCmd      uint8_t      Motorola         47        1         1         0         0         1         /     ||
         *   2       ADCU_Cld_DblFlashLampCmd  uint8_t      Motorola         46        1         1         0         0         1         /     ||
         *   3       ADCU_Cld_TurnLLampCmd     uint8_t      Motorola         45        1         1         0         0         1         /     ||
         *   4       ADCU_Cld_TurnRLampCmd     uint8_t      Motorola         44        1         1         0         0         1         /     ||
         *   5       ADCU_Cld_BackLampCmd      uint8_t      Motorola         43        1         1         0         0         1         /     ||
         *   6       ADCU_Cld_BuzzerCmd        uint8_t      Motorola         42        1         1         0         0         1         /     ||
         *   7       ADCU_Cld_HornCmd          uint8_t      Motorola         41        1         1         0         0         1         /     ||
         *   8       ADCU_Cld_RunLampCmd       uint8_t      Motorola         40        1         1         0         0         1         /     ||
         *=================================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Cld_HeadLampCmd", MOTOROLA, 47, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_headlampcmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_headlampcmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_headlampcmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_headlampcmd.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Cld_DblFlashLampCmd", MOTOROLA, 46, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_dblflashlampcmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_dblflashlampcmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_dblflashlampcmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_dblflashlampcmd.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Cld_TurnLLampCmd", MOTOROLA, 45, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_turnllampcmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_turnllampcmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_turnllampcmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_turnllampcmd.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Cld_TurnRLampCmd", MOTOROLA, 44, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_turnrlampcmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_turnrlampcmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_turnrlampcmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_turnrlampcmd.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Cld_BackLampCmd", MOTOROLA, 43, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_backlampcmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_backlampcmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_backlampcmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_backlampcmd.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Cld_BuzzerCmd", MOTOROLA, 42, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_buzzercmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_buzzercmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_buzzercmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_buzzercmd.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Cld_HornCmd", MOTOROLA, 41, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_horncmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_horncmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_horncmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_horncmd.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Cld_RunLampCmd", MOTOROLA, 40, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_runlampcmd.name,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_runlampcmd.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_runlampcmd.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_body_cmd_msg.adcu_cld_runlampcmd.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void CloudBodyCmd119::UpdateMsg(uint8_t *data, uint8_t len)
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
