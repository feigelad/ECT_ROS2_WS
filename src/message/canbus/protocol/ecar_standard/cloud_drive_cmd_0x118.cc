/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include "cloud_drive_cmd_0x118.h"

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t CloudDriveCmd118::Init(void *const msg)
      {
        /** Message Definition===============================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName         | @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       ADCU_Cld_CtrlAtive    uint8_t      Motorola         7         1         1         0         0         1         /     ||
         *   2       ADCU_Cld_PrkEnable    uint8_t      Motorola         6         1         1         0         0         1         /     ||
         *   3       ADCU_Cld_ThrotMode    uint8_t      Motorola         4         2         1         0         0         3         /     ||
         *   4       ADCU_Cld_TgtGear      uint8_t      Motorola         0         4         1         0         0         15        /     ||
         *   5       ADCU_Cld_TgtThrotVal  float        Motorola         8         8        0.4        0         0        100        %     ||
         *   6       ADCU_Cld_TgtBrkVal    float        Motorola         16        8        0.4        0         0        100        %     ||
         *   7       ADCU_Cld_TgtStrAngle  float        Motorola         32        16       0.1      -1000     -1000      1000      deg    ||
         *=================================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Cld_CtrlAtive", MOTOROLA, 7, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_ctrlative.name,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_ctrlative.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_ctrlative.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_ctrlative.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Cld_PrkEnable", MOTOROLA, 6, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_prkenable.name,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_prkenable.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_prkenable.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_prkenable.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Cld_ThrotMode", MOTOROLA, 4, 2, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_throtmode.name,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_throtmode.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_throtmode.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_throtmode.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Cld_TgtGear", MOTOROLA, 0, 4, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_tgtgear.name,
                                                           ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_tgtgear.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_tgtgear.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_tgtgear.value, 0, 15, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("ADCU_Cld_TgtThrotVal", MOTOROLA, 8, 8, 0.4, 0,
                                                       ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_tgtthrotval.name,
                                                       ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_tgtthrotval.unit,
                                                       &ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_tgtthrotval.time_stamp,
                                                       &ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_tgtthrotval.value, 0, 100, "%")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("ADCU_Cld_TgtBrkVal", MOTOROLA, 16, 8, 0.4, 0,
                                                       ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_tgtbrkval.name,
                                                       ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_tgtbrkval.unit,
                                                       &ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_tgtbrkval.time_stamp,
                                                       &ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_tgtbrkval.value, 0, 100, "%")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("ADCU_Cld_TgtStrAngle", MOTOROLA, 32, 16, 0.1, -1000,
                                                       ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_tgtstrangle.name,
                                                       ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_tgtstrangle.unit,
                                                       &ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_tgtstrangle.time_stamp,
                                                       &ecar_chassis_ptr_->cmd_msg.cloud_drive_cmd_msg.adcu_cld_tgtstrangle.value, -1000, 1000, "deg")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void CloudDriveCmd118::UpdateMsg(uint8_t *data, uint8_t len)
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
