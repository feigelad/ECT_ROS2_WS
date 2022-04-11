/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "../imu_ins570d/imu_stdcov_0x507.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t IMUStdCov507::Init(void *const msg)
      {
        /** Message Definition===========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName  |          @DataType  |  @ByteOrder | @StartBit | @BitLen | @Factor    | @Offset | @MinVal | @MaxVal | @Unit ||
         *   1       IMU_Ins_StdCov_Lat       float        Motorola       8          16       0.001          0         0       65.535     /   ||
         *   2       IMU_Ins_StdCov_Lon       float        Motorola       24         16       0.001          0         0       65.535     /   ||
         *   3       IMU_Ins_StdCov_Height    float        Motorola       40         16       0.001          0         0       65.535     /   ||
         *   4       IMU_Ins_StdCov_Heading   float        Motorola       56         16       0.001          0         0       65.535     /   ||
         *============================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_stdcov_stat_msg.header.frame_id = Name();

        if (false == AddSignal<float>(CanSignal<float>("IMU_Ins_StdCov_Lat", MOTOROLA, 8, 16, 0.001, 0,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_stdcov_stat_msg.imu_ins_stdcov_lat.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_stdcov_stat_msg.imu_ins_stdcov_lat.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_stdcov_stat_msg.imu_ins_stdcov_lat.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_stdcov_stat_msg.imu_ins_stdcov_lat.value, 0, 65.535, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("IMU_Ins_StdCov_Lon", MOTOROLA, 24, 16, 0.001, 0,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_stdcov_stat_msg.imu_ins_stdcov_lon.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_stdcov_stat_msg.imu_ins_stdcov_lon.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_stdcov_stat_msg.imu_ins_stdcov_lon.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_stdcov_stat_msg.imu_ins_stdcov_lon.value, 0, 65.535, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("IMU_Ins_StdCov_Height", MOTOROLA, 40, 16, 0.001, 0,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_stdcov_stat_msg.imu_ins_stdcov_height.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_stdcov_stat_msg.imu_ins_stdcov_height.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_stdcov_stat_msg.imu_ins_stdcov_height.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_stdcov_stat_msg.imu_ins_stdcov_height.value, 0, 65.535, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("IMU_Ins_StdCov_Heading", MOTOROLA, 56, 16, 0.001, 0,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_stdcov_stat_msg.imu_ins_stdcov_heading.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_stdcov_stat_msg.imu_ins_stdcov_heading.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_stdcov_stat_msg.imu_ins_stdcov_heading.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_stdcov_stat_msg.imu_ins_stdcov_heading.value, 0, 65.535, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void IMUStdCov507::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {
        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_stdcov_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_stdcov_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana