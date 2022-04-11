/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "remoter_stat1_0x270.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t RemoterStat270::Init(void *const msg)
      {
        /** Message Definition=========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName    |   @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_RmtChn1_Freq  float        Motorola         0         8         2         0         0        510       Hz    ||
         *   2       CDCU_RmtChn1_Duty  float        Motorola         16        16       0.01       0         0       655.35     %     ||
         *   3       CDCU_RmtChn2_Freq  float        Motorola         24         8         2         0         0        510      Hz    ||
         *   4       CDCU_RmtChn2_Duty  float        Motorola         40        16       0.01       0         0       655.35     %     ||
         *===========================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.remoter_stat1_msg.header.frame_id = Name();
        if (false == AddSignal<float>(CanSignal<float>("CDCU_RmtChn1_Freq", MOTOROLA, 0, 8, 2, 0,
                                                       ecar_chassis_ptr_->stat_msg.remoter_stat1_msg.cdcu_rmtchn1_freq.name,
                                                       ecar_chassis_ptr_->stat_msg.remoter_stat1_msg.cdcu_rmtchn1_freq.unit,
                                                       &ecar_chassis_ptr_->stat_msg.remoter_stat1_msg.cdcu_rmtchn1_freq.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.remoter_stat1_msg.cdcu_rmtchn1_freq.value, 0, 510, "Hz")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_RmtChn1_Duty", MOTOROLA, 16, 16, 0.01, 0,
                                                       ecar_chassis_ptr_->stat_msg.remoter_stat1_msg.cdcu_rmtchn1_duty.name,
                                                       ecar_chassis_ptr_->stat_msg.remoter_stat1_msg.cdcu_rmtchn1_duty.unit,
                                                       &ecar_chassis_ptr_->stat_msg.remoter_stat1_msg.cdcu_rmtchn1_duty.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.remoter_stat1_msg.cdcu_rmtchn1_duty.value, 0, 655.35, "%")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_RmtChn2_Freq", MOTOROLA, 24, 8, 2, 0,
                                                       ecar_chassis_ptr_->stat_msg.remoter_stat1_msg.cdcu_rmtchn2_freq.name,
                                                       ecar_chassis_ptr_->stat_msg.remoter_stat1_msg.cdcu_rmtchn2_freq.unit,
                                                       &ecar_chassis_ptr_->stat_msg.remoter_stat1_msg.cdcu_rmtchn2_freq.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.remoter_stat1_msg.cdcu_rmtchn2_freq.value, 0, 510, "Hz")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_RmtChn2_Duty", MOTOROLA, 40, 16, 0.01, 0,
                                                       ecar_chassis_ptr_->stat_msg.remoter_stat1_msg.cdcu_rmtchn2_duty.name,
                                                       ecar_chassis_ptr_->stat_msg.remoter_stat1_msg.cdcu_rmtchn2_duty.unit,
                                                       &ecar_chassis_ptr_->stat_msg.remoter_stat1_msg.cdcu_rmtchn2_duty.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.remoter_stat1_msg.cdcu_rmtchn2_duty.value, 0, 655.35, "%")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void RemoterStat270::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.remoter_stat1_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.remoter_stat1_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
