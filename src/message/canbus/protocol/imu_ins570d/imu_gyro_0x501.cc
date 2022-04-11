/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "../imu_ins570d/imu_gyro_0x501.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t IMUGyroStat501::Init(void *const msg)
      {
        /** Message Definition===========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName  |    @DataType  |  @ByteOrder | @StartBit | @BitLen | @Factor        | @Offset | @MinVal | @MaxVal | @Unit ||
         *   1       IMU_RateX         float          Motorola       8          16    0.0076293          -250      -250       250      deg/s   ||
         *   1       IMU_RateY         float          Motorola       24         16    0.0076293          -250      -250       250      deg/s   ||
         *   1       IMU_RateZ         float          Motorola       40         16    0.0076293          -250      -250       250      deg/s   ||
         *============================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_gyro_stat_msg.header.frame_id = Name();

        if (false == AddSignal<float>(CanSignal<float>("IMU_RateX", MOTOROLA, 8, 16, 0.0076293, -250,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_gyro_stat_msg.imu_ratex.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_gyro_stat_msg.imu_ratex.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_gyro_stat_msg.imu_ratex.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_gyro_stat_msg.imu_ratex.value, -250, 250, "deg/s")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("IMU_RateY", MOTOROLA, 24, 16, 0.0076293, -250,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_gyro_stat_msg.imu_ratey.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_gyro_stat_msg.imu_ratey.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_gyro_stat_msg.imu_ratey.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_gyro_stat_msg.imu_ratey.value, -250, 250, "deg/s")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("IMU_RateZ", MOTOROLA, 40, 16, 0.0076293, -250,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_gyro_stat_msg.imu_ratez.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_gyro_stat_msg.imu_ratez.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_gyro_stat_msg.imu_ratez.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_gyro_stat_msg.imu_ratez.value, -250, 250, "deg/s")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void IMUGyroStat501::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {
        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_gyro_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_gyro_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana