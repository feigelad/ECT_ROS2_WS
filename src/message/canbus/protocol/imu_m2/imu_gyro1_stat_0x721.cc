/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "../imu_m2/imu_gyro1_stat_0x721.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t IMUGyro1Stat721::Init(void *const msg)
      {
        /** Message Definition===========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName  |    @DataType  |  @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       IMU_RateX         float          Motorola       24         32    1/7000000      0         0        614     deg/s   ||
         *   2       IMU_RateY         float          Motorola       56         32    1/7000000      0         0        614     deg/s   ||
         *============================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_gyro1_stat_msg.header.frame_id = Name();

        if (false == AddSignal<float>(CanSignal<float>("IMU_RateX", MOTOROLA, 24, 32, 1 / 7000000.0, 0,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_gyro1_stat_msg.imu_ratex.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_gyro1_stat_msg.imu_ratex.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_gyro1_stat_msg.imu_ratex.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_gyro1_stat_msg.imu_ratex.value, 0, 614, "deg/s")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("IMU_RateY", MOTOROLA, 56, 32, 1 / 7000000.0, 0,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_gyro1_stat_msg.imu_ratey.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_gyro1_stat_msg.imu_ratey.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_gyro1_stat_msg.imu_ratey.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_gyro1_stat_msg.imu_ratey.value, 0, 614, "deg/s")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }
        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void IMUGyro1Stat721::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {
        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_gyro1_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_gyro1_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana