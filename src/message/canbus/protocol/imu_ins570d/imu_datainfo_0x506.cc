/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "../imu_ins570d/imu_datainfo_0x506.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t IMUDataInfoStat506::Init(void *const msg)
      {
        /** Message Definition===========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName  |         @DataType  |  @ByteOrder | @StartBit | @BitLen | @Factor    | @Offset | @MinVal | @MaxVal | @Unit  ||
         *   1       IMU_GpsFlag_Pos         uint8_t      Motorola        0          8         1             0         0       255       /    ||
         *   2       IMU_NumSV               uint8_t      Motorola        8          8         1             0         0       255       /    ||
         *   3       IMU_GpsFlag_Heading     uint8_t      Motorola        16         8         1             0         0       255       /    ||
         *   4       IMU_GpsAge              uint8_t      Motorola        24         8         1             0         0       255       /    ||
         *   5       IMU_CarStatus           uint8_t      Motorola        32         8         1             0         0       255       /    ||
         *   6       IMU_InsStatus           uint8_t      Motorola        56         8         1             0         0       255       /    ||
         *============================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.header.frame_id = Name();

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("IMU_GpsFlag_Pos", MOTOROLA, 0, 8, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_gpsflag_pos.name,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_gpsflag_pos.unit,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_gpsflag_pos.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_gpsflag_pos.value, 0, 255, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("IMU_NumSV", MOTOROLA, 8, 8, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_numsv.name,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_numsv.unit,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_numsv.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_numsv.value, 0, 255, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("IMU_GpsFlag_Heading", MOTOROLA, 16, 8, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_gpsflag_heading.name,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_gpsflag_heading.unit,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_gpsflag_heading.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_gpsflag_heading.value, 0, 255, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("IMU_GpsAge", MOTOROLA, 24, 8, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_gpsage.name,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_gpsage.unit,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_gpsage.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_gpsage.value, 0, 255, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("IMU_CarStatus", MOTOROLA, 32, 8, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_carstatus.name,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_carstatus.unit,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_carstatus.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_carstatus.value, 0, 255, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("IMU_InsStatus", MOTOROLA, 56, 8, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_insstatus.name,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_insstatus.unit,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_insstatus.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.imu_insstatus.value, 0, 255, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void IMUDataInfoStat506::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {
        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_datainfo_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana