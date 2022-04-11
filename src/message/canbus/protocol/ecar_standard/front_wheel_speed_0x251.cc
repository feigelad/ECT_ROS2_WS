/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "front_wheel_speed_0x251.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t FrontWheelSpd251::Init(void *const msg)
      {
        /** Message Definition==================================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName             |  @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_Veh_LfFtWhlSpd        float        Motorola        8          16       0.001       0         0       655.35   kmph    ||
         *   2       CDCU_Veh_RtFtWhlSpd        float        Motorola        24         16       0.001       0         0       655.35   kmph    ||
         *   3       CDCU_Veh_LfFtWhlSpdValid   uint8_t      Motorola        55         1          1         0         0         1       /      ||
         *   4       CDCU_Veh_RtFtWhlSpdValid   uint8_t      Motorola        54         1          1         0         0         1       /      ||
         *====================================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.front_wheel_spd_stat_msg.header.frame_id = Name();
        if (false == AddSignal<float>(CanSignal<float>("CDCU_Veh_LfFtWhlSpd", MOTOROLA, 8, 16, 0.001, 0,
                                                       ecar_chassis_ptr_->stat_msg.front_wheel_spd_stat_msg.cdcu_veh_lfwhlspd.name,
                                                       ecar_chassis_ptr_->stat_msg.front_wheel_spd_stat_msg.cdcu_veh_lfwhlspd.unit,
                                                       &ecar_chassis_ptr_->stat_msg.front_wheel_spd_stat_msg.cdcu_veh_lfwhlspd.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.front_wheel_spd_stat_msg.cdcu_veh_lfwhlspd.value, 0, 655.35, "kmph")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_Veh_RtFtWhlSpd", MOTOROLA, 24, 16, 0.001, 0,
                                                       ecar_chassis_ptr_->stat_msg.front_wheel_spd_stat_msg.cdcu_veh_rtwhlspd.name,
                                                       ecar_chassis_ptr_->stat_msg.front_wheel_spd_stat_msg.cdcu_veh_rtwhlspd.unit,
                                                       &ecar_chassis_ptr_->stat_msg.front_wheel_spd_stat_msg.cdcu_veh_rtwhlspd.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.front_wheel_spd_stat_msg.cdcu_veh_rtwhlspd.value, 0, 655.35, "kmph")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_Veh_LfFtWhlSpdValid", MOTOROLA, 55, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.front_wheel_spd_stat_msg.cdcu_veh_lfwhlspdvalid.name,
                                                           ecar_chassis_ptr_->stat_msg.front_wheel_spd_stat_msg.cdcu_veh_lfwhlspdvalid.unit,
                                                           &ecar_chassis_ptr_->stat_msg.front_wheel_spd_stat_msg.cdcu_veh_lfwhlspdvalid.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.front_wheel_spd_stat_msg.cdcu_veh_lfwhlspdvalid.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_Veh_RtFtWhlSpdValid", MOTOROLA, 54, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.front_wheel_spd_stat_msg.cdcu_veh_rtwhlspdvalid.name,
                                                           ecar_chassis_ptr_->stat_msg.front_wheel_spd_stat_msg.cdcu_veh_rtwhlspdvalid.unit,
                                                           &ecar_chassis_ptr_->stat_msg.front_wheel_spd_stat_msg.cdcu_veh_rtwhlspdvalid.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.front_wheel_spd_stat_msg.cdcu_veh_rtwhlspdvalid.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void FrontWheelSpd251::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.front_wheel_spd_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.front_wheel_spd_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
