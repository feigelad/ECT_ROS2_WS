/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include "steer_cmd_0x113.h"

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t SteerCmd113::Init(void *const msg)
      {
        /** Message Definition=========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName          | @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       ADCU_Str_Active        uint8_t      Motorola         7         1         1         0         0         1         /     ||
         *   2       ADCU_Str_CtrlMode      uint8_t      Motorola         5         2         1         0         0         3         /     ||
         *   3       ADCU_Str_TgtAngle      float        Motorola        16        16        0.005     -90       -90        90       deg    ||
         *   4       ADCU_Str_TgtCurvature  float        Motorola        32        16        0.0001    -3        -3         3        1/m    ||
         *   5       ADCU_Str_TgtAngleSpd   float        Motorola        40         8        0.2        0         0         51      deg/s   ||
         *===========================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Str_Active", MOTOROLA, 7, 1, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_active.name,
                                                           ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_active.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_active.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_active.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("ADCU_Str_CtrlMode", MOTOROLA, 5, 2, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_ctrlmode.name,
                                                           ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_ctrlmode.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_ctrlmode.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_ctrlmode.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("ADCU_Str_TgtAngle", MOTOROLA, 16, 16, 0.005, -90,
                                                       ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_tgtangle.name,
                                                       ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_tgtangle.unit,
                                                       &ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_tgtangle.time_stamp,
                                                       &ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_tgtangle.value, -90, 90, "deg")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("ADCU_Str_TgtCurvature", MOTOROLA, 32, 16, 0.0001, -3,
                                                       ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_tgtanglespd.name,
                                                       ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_tgtanglespd.unit,
                                                       &ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_tgtanglespd.time_stamp,
                                                       &ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_tgtanglespd.value, -3, 3, "1/m")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("ADCU_Str_TgtAngleSpd", MOTOROLA, 40, 8, 0.2, 0,
                                                       ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_tgtcurvature.name,
                                                       ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_tgtcurvature.unit,
                                                       &ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_tgtcurvature.time_stamp,
                                                       &ecar_chassis_ptr_->cmd_msg.steer_cmd_msg.adcu_str_tgtcurvature.value, 0, 51, "deg/s")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void SteerCmd113::UpdateMsg(uint8_t *data, uint8_t len)
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
