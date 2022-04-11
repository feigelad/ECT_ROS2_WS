/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "steer_diag_0x216.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t SteerDiag216::Init(void *const msg)
      {
        /** Message Definition=================================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName      |      @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_StrCmdPerd_Err     uint8_t      Motorola         7         1         1         0         0         1         /     ||
         *   2       CDCU_StrCmdDrop_Err     uint8_t      Motorola         6         1         1         0         0         1         /     ||
         *   3       CDCU_StrCmdChksum_Err   uint8_t      Motorola         5         1         1         0         0         1         /     ||
         *   4       CDCU_StrCmdOffline_Err  uint8_t      Motorola         4         1         1         0         0         1         /     ||
         *   5       CDCU_StrCmdOverRge_Err  uint8_t      Motorola         3         1         1         0         0         1         /     ||
         *   6       CDCU_EPSMsgPerd_Err     uint8_t      Motorola         15        1         1         0         0         1         /     ||
         *   7       CDCU_EPSMsgDrop_Err     uint8_t      Motorola         14        1         1         0         0         1         /     ||
         *   8       CDCU_EPSMsgChksum_Err   uint8_t      Motorola         13        1         1         0         0         1         /     ||
         *   9       CDCU_EPSMsgOffline_Err  uint8_t      Motorola         12        1         1         0         0         1         /     ||
         *===================================================================================================================================*/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.steer_diag_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_StrCmdPerd_Err", MOTOROLA, 7, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmdperd_err.name,
                                                           ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmdperd_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmdperd_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmdperd_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_StrCmdDrop_Err", MOTOROLA, 6, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmddrop_err.name,
                                                           ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmddrop_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmddrop_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmddrop_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_StrCmdChksum_Err", MOTOROLA, 5, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmdchksum_err.name,
                                                           ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmdchksum_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmdchksum_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmdchksum_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_StrCmdOffline_Err", MOTOROLA, 4, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmdoffline_err.name,
                                                           ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmdoffline_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmdoffline_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmdoffline_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_StrCmdOverRge_Err", MOTOROLA, 3, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmdoverrge_err.name,
                                                           ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmdoverrge_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmdoverrge_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_strcmdoverrge_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EPSMsgPerd_Err", MOTOROLA, 15, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_epsmsgperd_err.name,
                                                           ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_epsmsgperd_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_epsmsgperd_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_epsmsgperd_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EPSMsgDrop_Err", MOTOROLA, 14, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_epsmsgdrop_err.name,
                                                           ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_epsmsgdrop_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_epsmsgdrop_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_epsmsgdrop_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EPSMsgChksum_Err", MOTOROLA, 13, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_epsmsgchksum_err.name,
                                                           ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_epsmsgchksum_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_epsmsgchksum_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_epsmsgchksum_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EPSMsgOffline_Err", MOTOROLA, 12, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_epsmsgoffline_err.name,
                                                           ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_epsmsgoffline_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_epsmsgoffline_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.steer_diag_msg.cdcu_epsmsgoffline_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void SteerDiag216::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.steer_diag_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.steer_diag_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
