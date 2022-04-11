/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "../imu_m2/rtk_pos_cov_stat_0x707.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t RtkPosCovStat707::Init(void *const msg)
      {
        /** Message Definition=================================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName      |      @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       IMU_RtkPos_LonCov       float        Motorola        8          16      0.0001       0         0      6.5535     /      ||
         *   2       IMU_RtkPos_LatCov       float        Motorola        24         16      0.0001       0         0      6.5535     /      ||
         *   3       IMU_RtkPos_HeiCov       float        Motorola        40         16      0.0001       0         0      6.5535     /      ||
         *==================================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_pos_cov_stat_msg.header.frame_id = Name();

        if (false == AddSignal<float>(CanSignal<float>("IMU_RtkPos_LonCov", MOTOROLA, 8, 16, 0.0001, 0,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_pos_cov_stat_msg.imu_rtkpos_loncov.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_pos_cov_stat_msg.imu_rtkpos_loncov.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_pos_cov_stat_msg.imu_rtkpos_loncov.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_pos_cov_stat_msg.imu_rtkpos_loncov.value, 0, 6.5535, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("IMU_RtkPos_LatCov", MOTOROLA, 24, 16, 0.0001, 0,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_pos_cov_stat_msg.imu_rtkpos_latcov.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_pos_cov_stat_msg.imu_rtkpos_latcov.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_pos_cov_stat_msg.imu_rtkpos_latcov.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_pos_cov_stat_msg.imu_rtkpos_latcov.value, 0, 6.5535, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("IMU_RtkPos_HeiCov", MOTOROLA, 40, 16, 0.0001, 0,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_pos_cov_stat_msg.imu_rtkpos_heicov.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_pos_cov_stat_msg.imu_rtkpos_heicov.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_pos_cov_stat_msg.imu_rtkpos_heicov.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_pos_cov_stat_msg.imu_rtkpos_heicov.value, 0, 6.5535, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }
        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void RtkPosCovStat707::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {
        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_pos_cov_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.imu_rtk_pos_cov_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana