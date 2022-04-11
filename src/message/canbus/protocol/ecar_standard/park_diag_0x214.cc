/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "park_diag_0x214.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t ParkDiag214::Init(void *const msg)
      {
        /** Message Definition================================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName      |      @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_PrkCmdPerd_Err     uint8_t      Motorola         7         1         1         0         0         1         /     ||
         *   2       CDCU_PrkCmdDrop_Err     uint8_t      Motorola         6         1         1         0         0         1         /     ||
         *   3       CDCU_PrkCmdChksum_Err   uint8_t      Motorola         5         1         1         0         0         1         /     ||
         *   4       CDCU_PrkCmdOffline_Err  uint8_t      Motorola         4         1         1         0         0         1         /     ||
         *   5       CDCU_PrkCmdOverRge_Err  uint8_t      Motorola         3         1         1         0         0         1         /     ||
         *   6       CDCU_EPBMsgPerd_Err     uint8_t      Motorola         15        1         1         0         0         1         /     ||
         *   7       CDCU_EPBMsgDrop_Err     uint8_t      Motorola         14        1         1         0         0         1         /     ||
         *   8       CDCU_EPBMsgChksum_Err   uint8_t      Motorola         13        1         1         0         0         1         /     ||
         *   9       CDCU_EPBMsgOffline_Err  uint8_t      Motorola         12        1         1         0         0         1         /     ||
         *===================================================================================================================================*/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.park_diag_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_PrkCmdPerd_Err", MOTOROLA, 7, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmdperd_err.name,
                                                           ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmdperd_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmdperd_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmdperd_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_PrkCmdDrop_Err", MOTOROLA, 6, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmddrop_err.name,
                                                           ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmddrop_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmddrop_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmddrop_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_PrkCmdChksum_Err", MOTOROLA, 5, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmdchksum_err.name,
                                                           ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmdchksum_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmdchksum_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmdchksum_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_PrkCmdOffline_Err", MOTOROLA, 4, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmdoffline_err.name,
                                                           ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmdoffline_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmdoffline_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmdoffline_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_PrkCmdOverRge_Err", MOTOROLA, 3, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmdoverrge_err.name,
                                                           ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmdoverrge_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmdoverrge_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_prkcmdoverrge_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EPBMsgPerd_Err", MOTOROLA, 15, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_epbmsgperd_err.name,
                                                           ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_epbmsgperd_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_epbmsgperd_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_epbmsgperd_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EPBMsgDrop_Err", MOTOROLA, 14, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_epbmsgdrop_err.name,
                                                           ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_epbmsgdrop_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_epbmsgdrop_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_epbmsgdrop_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EPBMsgChksum_Err", MOTOROLA, 13, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_epbmsgchksum_err.name,
                                                           ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_epbmsgchksum_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_epbmsgchksum_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_epbmsgchksum_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EPBMsgOffline_Err", MOTOROLA, 12, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_epbmsgoffline_err.name,
                                                           ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_epbmsgoffline_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_epbmsgoffline_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.park_diag_msg.cdcu_epbmsgoffline_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void ParkDiag214::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.park_diag_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.park_diag_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
