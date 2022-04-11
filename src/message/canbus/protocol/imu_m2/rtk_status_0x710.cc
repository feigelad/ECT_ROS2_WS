/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "../imu_m2/rtk_status_0x710.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t RtkStat710::Init(void *const msg)
      {
        /** Message Definition===========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName  |    @DataType  |  @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       IMU_RtkStatus     uint8_t        Motorola        0          8          1        0         0        12       /      ||
         *   2       IMU_DmiStatus     uint8_t        Motorola        8          8          1        0         0        1        /      ||
         *   3       IMU_GpsStatus     uint8_t        Motorola        16         8          1        0         0        80       /      ||
         *============================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_status_msg.header.frame_id = Name();

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("IMU_RtkStatus", MOTOROLA, 0, 8, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_status_msg.imu_rtkstatus.name,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_status_msg.imu_rtkstatus.unit,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_status_msg.imu_rtkstatus.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_status_msg.imu_rtkstatus.value, 0, 12, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("IMU_DmiStatus", MOTOROLA, 8, 8, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_status_msg.imu_dmistatus.name,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_status_msg.imu_dmistatus.unit,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_status_msg.imu_dmistatus.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_status_msg.imu_dmistatus.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("IMU_GpsStatus", MOTOROLA, 16, 8, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_status_msg.imu_gpsstatus.name,
                                                           ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_status_msg.imu_gpsstatus.unit,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_status_msg.imu_gpsstatus.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_status_msg.imu_gpsstatus.value, 0, 80, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }
        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void RtkStat710::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {
        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_status_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_status_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana