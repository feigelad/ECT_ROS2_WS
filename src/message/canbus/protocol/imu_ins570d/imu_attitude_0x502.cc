/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "../imu_ins570d/imu_attitude_0x502.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t IMUAttStat502::Init(void *const msg)
      {
        /** Message Definition===========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName  |         @DataType  |  @ByteOrder | @StartBit | @BitLen | @Factor    | @Offset | @MinVal | @MaxVal | @Unit ||
         *   1       IMU_RtkAtt_Pitch        float        Motorola        8          16      0.010986     -360      -360       360      deg  ||
         *   2       IMU_RtkAtt_Roll         float        Motorola        24         16      0.010986     -360      -360       360      deg  ||
         *   3       IMU_RtkAtt_Yaw          float        Motorola        40         16      0.010986     -360      -360       360      deg  ||
         *============================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_attitude_stat_msg.header.frame_id = Name();

        if (false == AddSignal<float>(CanSignal<float>("IMU_RtkAtt_Pitch", MOTOROLA, 8, 16, 0.010986, -360,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_attitude_stat_msg.imu_rtkatt_pitch.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_attitude_stat_msg.imu_rtkatt_pitch.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_attitude_stat_msg.imu_rtkatt_pitch.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_attitude_stat_msg.imu_rtkatt_pitch.value, -360, 360, "deg")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("IMU_RtkAtt_Roll", MOTOROLA, 24, 16, 0.010986, -360,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_attitude_stat_msg.imu_rtkatt_roll.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_attitude_stat_msg.imu_rtkatt_roll.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_attitude_stat_msg.imu_rtkatt_roll.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_attitude_stat_msg.imu_rtkatt_roll.value, -360, 360, "deg")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("IMU_RtkAtt_Yaw", MOTOROLA, 40, 16, 0.010986, -360,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_attitude_stat_msg.imu_rtkatt_yaw.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_attitude_stat_msg.imu_rtkatt_yaw.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_attitude_stat_msg.imu_rtkatt_yaw.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_attitude_stat_msg.imu_rtkatt_yaw.value, -360, 360, "deg")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void IMUAttStat502::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {
        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_attitude_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_attitude_stat_msg.header.stamp = clock.now();
        // std::cout << "TimeStamp:" << ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_attitude_stat_msg.header.stamp
        //           << ", IMU_RtkAtt_Pitch:" << ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_attitude_stat_msg.IMU_RtkAtt_Pitch.value
        //           << ", IMU_RtkAtt_Roll:" << ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_attitude_stat_msg.IMU_RtkAtt_Roll.value
        //           << ", IMU_RtkAtt_Yaw:" << ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_attitude_stat_msg.IMU_RtkAtt_Yaw.value
        //           << std::endl;
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana