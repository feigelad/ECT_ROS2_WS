/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */

#include <rclcpp/rclcpp.hpp>
#include "bat_stat_0x222.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t BatStat222::Init(void *const msg)
      {
        /** Message Definition=========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName    |     @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_LvBat_Volt      float        Motorola         0         8        0.1        0         0        25.5       V     ||
         *   2       CDCU_BMS_BatSOC      float        Motorola         8         8        0.4        0         0        100        %     ||
         *   3       CDCU_BMS_BatSOH      float        Motorola         16        8        0.4        0         0        100        %     ||
         *   4       CDCU_BMS_BatVolt     float        Motorola         24        8        0.4        0         0        100        V     ||
         *   5       CDCU_BMS_BatCurt     float        Motorola         32        8         1         0         0        255        A     ||
         *   6       CDCU_BMS_BatTemp     float        Motorola         40        8         1        -50       -50       205        ℃    ||
         *   7       CDCU_BMS_ChgSt       uint8_t      Motorola         55        1         1         0         0         1         /     ||
         *   8       CDCU_BMS_DisChgSt    uint8_t      Motorola         54        1         1         0         0         1         /     ||
         *   9       CDCU_BMS_FeedBackSt  uint8_t      Motorola         53        1         1         0         0         1         /     ||
         *===========================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.bat_stat_msg.header.frame_id = Name();

        if (false == AddSignal<float>(CanSignal<float>("CDCU_LvBat_Volt", MOTOROLA, 0, 8, 0.1, 0,
                                                       ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_lvbat_volt.name,
                                                       ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_lvbat_volt.unit,
                                                       &ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_lvbat_volt.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_lvbat_volt.value, 0, 25.5, "V")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_BMS_BatSOC", MOTOROLA, 8, 8, 0.4, 0,
                                                       ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_batsoc.name,
                                                       ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_batsoc.unit,
                                                       &ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_batsoc.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_batsoc.value, 0, 100, "%")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_BMS_BatSOH", MOTOROLA, 16, 8, 0.4, 0,
                                                       ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_batsoh.name,
                                                       ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_batsoh.unit,
                                                       &ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_batsoh.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_batsoh.value, 0, 100, "%")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_BMS_BatVolt", MOTOROLA, 24, 8, 0.4, 0,
                                                       ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_batvolt.name,
                                                       ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_batvolt.unit,
                                                       &ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_batvolt.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_batvolt.value, 0, 100, "V")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_BMS_BatCurt", MOTOROLA, 32, 8, 1, 0,
                                                       ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_batcurt.name,
                                                       ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_batcurt.unit,
                                                       &ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_batcurt.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_batcurt.value, 0, 255, "A")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_BMS_BatTemp", MOTOROLA, 40, 8, 1, -50,
                                                       ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_battemp.name,
                                                       ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_battemp.unit,
                                                       &ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_battemp.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_battemp.value, -50, 205, "℃")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMS_ChgSt", MOTOROLA, 55, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_chgst.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_chgst.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_chgst.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_chgst.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMS_DisChgSt", MOTOROLA, 54, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_dischgst.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_dischgst.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_dischgst.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_dischgst.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMS_FeedBackSt", MOTOROLA, 53, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_feedbackst.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_feedbackst.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_feedbackst.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_stat_msg.cdcu_bms_feedbackst.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }
        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void BatStat222::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.bat_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.bat_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
