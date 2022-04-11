/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "../imu_m2/rtk_attitude_stat_0x706.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t RtkAttitudeStat706::Init(void *const msg)
      {
        /** Message Definition=================================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName      |      @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       IMU_RtkAtt_Yaw          float        Motorola        20         20      0.001       0         0        360      deg     ||
         *   2       IMU_RtkAtt_Pitch        float        Motorola        40         20      0.001      -90       -90       90       deg     ||
         *   3       IMU_RtkAtt_Roll         float        Motorola        60         20      0.001      -90       -90       90       deg     ||
         *==================================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_attitude_stat_msg.header.frame_id = Name();

        if (false == AddSignal<float>(CanSignal<float>("IMU_RtkAtt_Yaw", MOTOROLA, 20, 20, 0.001, 0,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_attitude_stat_msg.imu_rtkatt_yaw.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_attitude_stat_msg.imu_rtkatt_yaw.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_attitude_stat_msg.imu_rtkatt_yaw.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_attitude_stat_msg.imu_rtkatt_yaw.value, 0, 360, "deg")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("IMU_RtkAtt_Pitch", MOTOROLA, 40, 20, 0.001, -90,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_attitude_stat_msg.imu_rtkatt_pitch.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_attitude_stat_msg.imu_rtkatt_pitch.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_attitude_stat_msg.imu_rtkatt_pitch.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_attitude_stat_msg.imu_rtkatt_pitch.value, -90, 90, "deg")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("IMU_RtkAtt_Roll", MOTOROLA, 60, 20, 0.001, -90,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_attitude_stat_msg.imu_rtkatt_roll.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_attitude_stat_msg.imu_rtkatt_roll.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_attitude_stat_msg.imu_rtkatt_roll.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_attitude_stat_msg.imu_rtkatt_roll.value, -90, 90, "deg")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }
        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void RtkAttitudeStat706::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {
        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_attitude_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_attitude_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana