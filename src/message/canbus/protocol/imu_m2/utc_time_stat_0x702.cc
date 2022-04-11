/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "../imu_m2/utc_time_stat_0x702.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t UtcTimeStat702::Init(void *const msg)
      {
        /** Message Definition================================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName         |   @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       IMU_UtcTime_Year        uint16_t      Motorola        8        16         1         0         0       65535      /      ||
         *   2       IMU_UtcTime_Month       uint8_t       Motorola       16         8         1         0         1        12        /      ||
         *   3       IMU_UtcTime_Date        uint8_t       Motorola       24         8         1         0         1        31        /      ||
         *   4       IMU_UtcTime_Hour        uint8_t       Motorola       32         8         1         0         0        24        /      ||
         *   5       IMU_UtcTime_Min         uint8_t       Motorola       40         8         1         0         0        60        /      ||
         *   6       IMU_UtcTime_IntSec      uint8_t       Motorola       48         8         1         0         0        59        /      ||
         *   7       IMU_UtcTime_DotSec      float         Motorola       56         8        0.01       0         0       0.99       /      ||
         *==================================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.header.frame_id = Name();

        if (false == AddSignal<uint16_t>(CanSignal<uint16_t>("IMU_UtcTime_Year", MOTOROLA, 8, 16, 1, 0,
                                                             ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_year.name,
                                                             ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_year.unit,
                                                             &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_year.time_stamp,
                                                             &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_year.value, 0, 65535, "year")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("IMU_UtcTime_Month", MOTOROLA, 16, 8, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_month.name,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_month.unit,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_month.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_month.value, 1, 12, "month")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("IMU_UtcTime_Date", MOTOROLA, 24, 8, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_date.name,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_date.unit,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_date.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_date.value, 1, 31, "date")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("IMU_UtcTime_Hour", MOTOROLA, 32, 8, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_hour.name,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_hour.unit,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_hour.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_hour.value, 0, 24, "hour")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("IMU_UtcTime_Min", MOTOROLA, 40, 8, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_min.name,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_min.unit,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_min.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_min.value, 0, 60, "min")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("IMU_UtcTime_IntSec", MOTOROLA, 48, 8, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_intsec.name,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_intsec.unit,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_intsec.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_intsec.value, 0, 59, "int_sec")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("IMU_UtcTime_DotSec", MOTOROLA, 56, 8, 0.01, 0,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_dotsec.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_dotsec.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_dotsec.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.imu_utctime_dotsec.value, 0, 0.99, "dot_sec")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }
        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void UtcTimeStat702::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_utc_time_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
