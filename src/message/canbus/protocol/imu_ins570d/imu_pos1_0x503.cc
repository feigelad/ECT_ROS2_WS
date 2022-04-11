/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "../imu_ins570d/imu_pos1_0x503.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t IMUPos1Stat503::Init(void *const msg)
      {
        /** Message Definition===========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName  |         @DataType  |  @ByteOrder | @StartBit | @BitLen | @Factor    | @Offset | @MinVal | @MaxVal | @Unit ||
         *   1       IMU_RtkPos_Height       double       Motorola        24         32      0.001        -10000      0       100000     /   ||
         *   2       IMU_WeekTime_WeekSecs   double       Motorola        56         32      0.001          0         0      4294970     /   ||
         *============================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_pos1_stat_msg.header.frame_id = Name();

        if (false == AddSignal<double>(CanSignal<double>("IMU_RtkPos_Height", MOTOROLA, 24, 32, 0.001, -10000,
                                                         ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_pos1_stat_msg.imu_rtkpos_height.name,
                                                         ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_pos1_stat_msg.imu_rtkpos_height.unit,
                                                         &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_pos1_stat_msg.imu_rtkpos_height.time_stamp,
                                                         &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_pos1_stat_msg.imu_rtkpos_height.value, -10000, 10000, "m")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<double>(CanSignal<double>("IMU_WeekTime_WeekSecs", MOTOROLA, 56, 32, 0.001, 0,
                                                         ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_pos1_stat_msg.imu_weektime_weeksecs.name,
                                                         ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_pos1_stat_msg.imu_weektime_weeksecs.unit,
                                                         &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_pos1_stat_msg.imu_weektime_weeksecs.time_stamp,
                                                         &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_pos1_stat_msg.imu_weektime_weeksecs.value, 0, 4294970, "s")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void IMUPos1Stat503::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {
        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_pos1_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_pos1_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana