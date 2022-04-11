/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "../imu_m2/week_time_stat_0x701.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t WeekTimeStat701::Init(void *const msg)
      {
        /** Message Definition================================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName         |   @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       IMU_WeekTime_Weeks      uint16_t      Motorola        8        16         1         0         0       65535      /     ||
         *   2       IMU_WeekTime_WeekSecs   double        Motorola       56        48       0.001       0       604800      1        /     ||
         *==================================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_week_time_stat_msg.header.frame_id = Name();

        if (false == AddSignal<uint16_t>(CanSignal<uint16_t>("IMU_WeekTime_Weeks", MOTOROLA, 8, 16, 1, 0,
                                                             ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_week_time_stat_msg.imu_weektime_weeks.name,
                                                             ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_week_time_stat_msg.imu_weektime_weeks.unit,
                                                             &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_week_time_stat_msg.imu_weektime_weeks.time_stamp,
                                                             &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_week_time_stat_msg.imu_weektime_weeks.value, 0, 65535, "weeks")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<double>(CanSignal<double>("IMU_WeekTime_WeekSecs", MOTOROLA, 56, 48, 0.001, 0,
                                                         ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_week_time_stat_msg.imu_weektime_weeksecs.name,
                                                         ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_week_time_stat_msg.imu_weektime_weeksecs.unit,
                                                         &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_week_time_stat_msg.imu_weektime_weeksecs.time_stamp,
                                                         &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_week_time_stat_msg.imu_weektime_weeksecs.value, 0, 604800, "s")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }
        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void WeekTimeStat701::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_week_time_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_week_time_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
