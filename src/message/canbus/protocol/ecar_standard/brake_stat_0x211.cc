/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "brake_stat_0x211.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t BrakeStat211::Init(void *const msg)
      {
        /** Message Definition=========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName    |   @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_EHB_WorkMode  uint8_t      Motorola         4         4         1         0         0         15         /     ||
         *   2       CDCU_EHB_BrkMode   uint8_t      Motorola         2         2         1         0         0         3         /     ||
         *   3       CDCU_EHB_BrkPresur float        Motorola         8         8         1         0         0        255       bar    ||
         *   4       CDCU_EHB_ErrLevel  uint8_t      Motorola         52        4         1         0         0         15         /     ||
         *===========================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.brake_stat_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EHB_WorkMode", MOTOROLA, 4, 4, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_stat_msg.cdcu_ehb_workmode.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_stat_msg.cdcu_ehb_workmode.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_stat_msg.cdcu_ehb_workmode.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_stat_msg.cdcu_ehb_workmode.value, 0, 15, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EHB_BrkMode", MOTOROLA, 2, 2, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_stat_msg.cdcu_ehb_brkmode.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_stat_msg.cdcu_ehb_brkmode.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_stat_msg.cdcu_ehb_brkmode.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_stat_msg.cdcu_ehb_brkmode.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_EHB_BrkPresur", MOTOROLA, 8, 8, 1, 0,
                                                       ecar_chassis_ptr_->stat_msg.brake_stat_msg.cdcu_ehb_brkpresur.name,
                                                       ecar_chassis_ptr_->stat_msg.brake_stat_msg.cdcu_ehb_brkpresur.unit,
                                                       &ecar_chassis_ptr_->stat_msg.brake_stat_msg.cdcu_ehb_brkpresur.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.brake_stat_msg.cdcu_ehb_brkpresur.value, 0, 255, "bar")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EHB_ErrLevel", MOTOROLA, 52, 4, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_stat_msg.cdcu_ehb_errlevel.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_stat_msg.cdcu_ehb_errlevel.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_stat_msg.cdcu_ehb_errlevel.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_stat_msg.cdcu_ehb_errlevel.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void BrakeStat211::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.brake_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.brake_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
