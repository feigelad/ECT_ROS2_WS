/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "../imu_ins570d/imu_speed_0x505.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t IMUSpeedStat505::Init(void *const msg)
      {
        /** Message Definition===========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName  |         @DataType  |  @ByteOrder | @StartBit | @BitLen | @Factor    | @Offset | @MinVal | @MaxVal | @Unit  ||
         *   1       IMU_RtkSpeed_North      float        Motorola        8          16     0.030517      -100      -100       100      mps   ||
         *   2       IMU_RtkSpeed_East       float        Motorola        24         16     0.030517      -100      -100       100      mps   ||
         *   3       IMU_RtkSpeed_Horiz      float        Motorola        40         16     0.030517      -100      -100       100      mps   ||
         *============================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_speed_stat_msg.header.frame_id = Name();

        if (false == AddSignal<float>(CanSignal<float>("IMU_RtkSpeed_North", MOTOROLA, 8, 16, 0.030517, -100,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_speed_stat_msg.imu_rtkspeed_north.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_speed_stat_msg.imu_rtkspeed_north.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_speed_stat_msg.imu_rtkspeed_north.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_speed_stat_msg.imu_rtkspeed_north.value, -100, 100, "mps")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("IMU_RtkSpeed_East", MOTOROLA, 24, 16, 0.030517, -100,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_speed_stat_msg.imu_rtkspeed_east.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_speed_stat_msg.imu_rtkspeed_east.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_speed_stat_msg.imu_rtkspeed_east.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_speed_stat_msg.imu_rtkspeed_east.value, -100, 100, "mps")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("IMU_RtkSpeed_Horiz", MOTOROLA, 40, 16, 0.030517, -100,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_speed_stat_msg.imu_rtkspeed_horiz.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_speed_stat_msg.imu_rtkspeed_horiz.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_speed_stat_msg.imu_rtkspeed_horiz.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_speed_stat_msg.imu_rtkspeed_horiz.value, -100, 100, "mps")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void IMUSpeedStat505::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {
        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_speed_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_speed_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana