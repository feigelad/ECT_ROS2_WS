/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "../imu_ins570d/imu_acc_0x500.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t IMUAccStat500::Init(void *const msg)
      {
        /** Message Definition===========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName  |    @DataType  |  @ByteOrder | @StartBit | @BitLen | @Factor        | @Offset | @MinVal | @MaxVal | @Unit ||
         *   1       IMU_AccelX        float          Motorola       8          16    0.0001220703125      -4        -4        4        g   ||
         *   2       IMU_AccelY        float          Motorola       24         16    0.0001220703125      -4        -4        4        g   ||
         *   3       IMU_AccelZ        float          Motorola       40         16    0.0001220703125      -4        -4        4        g   ||
         *============================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_acc_stat_msg.header.frame_id = Name();

        if (false == AddSignal<float>(CanSignal<float>("IMU_AccelX", MOTOROLA, 8, 16, 0.0001220703125, -4,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_acc_stat_msg.imu_accelx.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_acc_stat_msg.imu_accelx.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_acc_stat_msg.imu_accelx.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_acc_stat_msg.imu_accelx.value, -4, 4, "g")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("IMU_AccelY", MOTOROLA, 24, 16, 0.0001220703125, -4,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_acc_stat_msg.imu_accely.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_acc_stat_msg.imu_accely.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_acc_stat_msg.imu_accely.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_acc_stat_msg.imu_accely.value, -4, 4, "g")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("IMU_AccelZ", MOTOROLA, 40, 16, 0.0001220703125, -4,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_acc_stat_msg.imu_accelz.name,
                                                       ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_acc_stat_msg.imu_accelz.unit,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_acc_stat_msg.imu_accelz.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_acc_stat_msg.imu_accelz.value, -4, 4, "g")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void IMUAccStat500::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {
        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_acc_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_acc_stat_msg.header.stamp = clock.now();
        // std::cout << "TimeStamp:" << ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_acc_stat_msg.header.stamp
        //           << ", IMU_AccelX:" << ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_acc_stat_msg.IMU_AccelX.value
        //           << ", IMU_AccelY:" << ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_acc_stat_msg.IMU_AccelY.value
        //           << ", IMU_AccelZ:" << ecar_chassis_ptr_->stat_msg.imu_stat_msg.ins570d_acc_stat_msg.IMU_AccelZ.value
        //           << std::endl;
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana