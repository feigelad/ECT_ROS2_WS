/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "../imu_m2/imu_input_cmd_0x099.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t IMUInputCmd099::Init(void *const msg)
      {
        /** Message Definition====================================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName           |    @DataType  |  @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_Veh_RunDirect         uint8_t        Motorola       5          3         1         0         0        2         /      ||
         *   2       CDCU_Veh_SpeedKmph         float          Motorola       16         16      1/256       0         0        256      kmph    ||
         *======================================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->cmd_msg.imu_input_cmd_msg.header.frame_id = Name();

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_Veh_RunDirect", MOTOROLA, 5, 3, 1, 0,
                                                           ecar_chassis_ptr_->cmd_msg.imu_input_cmd_msg.cdcu_veh_rundirect.name,
                                                           ecar_chassis_ptr_->cmd_msg.imu_input_cmd_msg.cdcu_veh_rundirect.unit,
                                                           &ecar_chassis_ptr_->cmd_msg.imu_input_cmd_msg.cdcu_veh_rundirect.time_stamp,
                                                           &ecar_chassis_ptr_->cmd_msg.imu_input_cmd_msg.cdcu_veh_rundirect.value, 0, 2, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_Veh_SpeedKmph", MOTOROLA, 16, 16, 1 / 256.0, 0,
                                                       ecar_chassis_ptr_->cmd_msg.imu_input_cmd_msg.cdcu_veh_speedkmph.name,
                                                       ecar_chassis_ptr_->cmd_msg.imu_input_cmd_msg.cdcu_veh_speedkmph.unit,
                                                       &ecar_chassis_ptr_->cmd_msg.imu_input_cmd_msg.cdcu_veh_speedkmph.time_stamp,
                                                       &ecar_chassis_ptr_->cmd_msg.imu_input_cmd_msg.cdcu_veh_speedkmph.value, 0, 256, "kmph")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void IMUInputCmd099::UpdateMsg(uint8_t *data, uint8_t len)
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