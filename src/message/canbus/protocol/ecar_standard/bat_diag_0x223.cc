/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */

#include <rclcpp/rclcpp.hpp>
#include "bat_diag_0x223.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t BatDiag223::Init(void *const msg)
      {
        /** Message Definition=======================================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName          |        @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_BMSMsgPerd_Err           uint8_t      Motorola         7         1         1         0         0         1         /     ||
         *   2       CDCU_BMSMsgDrop_Err           uint8_t      Motorola         6         1         1         0         0         1         /     ||
         *   3       CDCU_BMSMsgChksum_Err         uint8_t      Motorola         5         1         1         0         0         1         /     ||
         *   4       CDCU_BMSMsgOffline_Err        uint8_t      Motorola         4         1         1         0         0         1         /     ||
         *   5       CDCU_BMSLowSOC_Alarm          uint8_t      Motorola         15        1         1         0         0         1         /     ||
         *   6       CDCU_BMSLowSOC_Err            uint8_t      Motorola         14        1         1         0         0         1         /     ||
         *   7       CDCU_BMSOverVolt_Err          uint8_t      Motorola         13        1         1         0         0         1         /     ||
         *   8       CDCU_BMSUnderVolt_Err         uint8_t      Motorola         12        1         1         0         0         1         /     ||
         *   9       CDCU_BMSOverCurt_Err          uint8_t      Motorola         11        1         1         0         0         1         /     ||
         *   10      CDCU_BMSOverTemp_Err          uint8_t      Motorola         10        1         1         0         0         1         /     ||
         *   11      CDCU_BMSUnitUnderVolt_Err     uint8_t      Motorola         9         1         1         0         0         1         /     ||
         *   12      CDCU_BMSUnitOverVolt_Err      uint8_t      Motorola         8         1         1         0         0         1         /     ||
         *   13      CDCU_BMSUnitOverCurt_Err      uint8_t      Motorola         23        1         1         0         0         1         /     ||
         *   14      CDCU_BMSUnitOverTemp_Err      uint8_t      Motorola         22        1         1         0         0         1         /     ||
         *   15      CDCU_BMSLvBatUnderVolt_Alarm  uint8_t      Motorola         21        1         1         0         0         1         /     ||
         *   16      CDCU_BMSLvBatUnderVolt_Err    uint8_t      Motorola         20        1         1         0         0         1         /     ||
         *   17      CDCU_BMSLvBatOverVolt_Alarm   uint8_t      Motorola         19        1         1         0         0         1         /     ||
         *=========================================================================================================================================*/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.bat_diag_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMSMsgPerd_Err", MOTOROLA, 7, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsmsgperd_err.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsmsgperd_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsmsgperd_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsmsgperd_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMSMsgDrop_Err", MOTOROLA, 6, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsmsgdrop_err.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsmsgdrop_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsmsgdrop_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsmsgdrop_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMSMsgChksum_Err", MOTOROLA, 5, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsmsgchksum_err.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsmsgchksum_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsmsgchksum_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsmsgchksum_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMSMsgOffline_Err", MOTOROLA, 4, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsmsgoffline_err.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsmsgoffline_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsmsgoffline_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsmsgoffline_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMSLowSOC_Alarm", MOTOROLA, 15, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslowsoc_alarm.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslowsoc_alarm.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslowsoc_alarm.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslowsoc_alarm.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMSLowSOC_Err", MOTOROLA, 14, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslowsoc_err.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslowsoc_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslowsoc_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslowsoc_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMSOverVolt_Err", MOTOROLA, 13, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsovervolt_err.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsovervolt_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsovervolt_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsovervolt_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMSUnderVolt_Err", MOTOROLA, 12, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsundervolt_err.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsundervolt_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsundervolt_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsundervolt_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMSOverCurt_Err", MOTOROLA, 11, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsovercurt_err.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsovercurt_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsovercurt_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsovercurt_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMSOverTemp_Err", MOTOROLA, 10, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsovertemp_err.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsovertemp_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsovertemp_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsovertemp_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMSUnitUnderVolt_Err", MOTOROLA, 9, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsunitundervolt_err.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsunitundervolt_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsunitundervolt_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsunitundervolt_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMSUnitOverVolt_Err", MOTOROLA, 8, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsunitovervolt_err.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsunitovervolt_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsunitovervolt_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsunitovervolt_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMSUnitOverCurt_Err", MOTOROLA, 23, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsunitovercurt_err.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsunitovercurt_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsunitovercurt_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsunitovercurt_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMSUnitOverTemp_Err", MOTOROLA, 22, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsunitovertemp_err.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsunitovertemp_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsunitovertemp_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmsunitovertemp_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMSLvBatUnderVolt_Alarm", MOTOROLA, 21, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslvbatundervolt_alarm.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslvbatundervolt_alarm.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslvbatundervolt_alarm.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslvbatundervolt_alarm.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMSLvBatUnderVolt_Err", MOTOROLA, 20, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslvbatundervolt_err.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslvbatundervolt_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslvbatundervolt_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslvbatundervolt_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMSLvBatOverVolt_Alarm", MOTOROLA, 19, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslvbatovervolt_alarm.name,
                                                           ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslvbatovervolt_alarm.unit,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslvbatovervolt_alarm.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.bat_diag_msg.cdcu_bmslvbatovervolt_alarm.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void BatDiag223::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {
        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.bat_diag_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.bat_diag_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
