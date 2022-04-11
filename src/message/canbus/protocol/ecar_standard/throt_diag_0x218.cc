/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "throt_diag_0x218.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t ThrotDiag218::Init(void *const msg)
      {
        /** Message Definition==================================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName        |      @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_DrvCmdPerd_Err       uint8_t      Motorola         7         1         1         0         0         1         /     ||
         *   2       CDCU_DrvCmdDrop_Err       uint8_t      Motorola         6         1         1         0         0         1         /     ||
         *   3       CDCU_DrvCmdChksum_Err     uint8_t      Motorola         5         1         1         0         0         1         /     ||
         *   4       CDCU_DrvCmdOffline_Err    uint8_t      Motorola         4         1         1         0         0         1         /     ||
         *   5       CDCU_DrvCmdOverRge_Err    uint8_t      Motorola         3         1         1         0         0         1         /     ||
         *   6       CDCU_MCUMsgPerd_Err       uint8_t      Motorola         15        1         1         0         0         1         /     ||
         *   7       CDCU_MCUMsgDrop_Err       uint8_t      Motorola         14        1         1         0         0         1         /     ||
         *   8       CDCU_MCUMsgChksum_Err     uint8_t      Motorola         13        1         1         0         0         1         /     ||
         *   9       CDCU_MCUMsgOffline_Err    uint8_t      Motorola         12        1         1         0         0         1         /     ||
         *   10      CDCU_MCUMainPwrCur_Err    uint8_t      Motorola         23        1         1         0         0         1         /     ||
         *   11      CDCU_MCUCurtSensor_Err    uint8_t      Motorola         22        1         1         0         0         1         /     ||
         *   12      CDCU_MCUPreChg_Err        uint8_t      Motorola         21        1         1         0         0         1         /     ||
         *   13      CDCU_MCUPwrOverVolt_Err   uint8_t      Motorola         20        1         1         0         0         1         /     ||
         *   14      CDCU_MCUPwrUnderVolt_Err  uint8_t      Motorola         19        1         1         0         0         1         /     ||
         *   15      CDCU_MCUPwrOverCurt_Err   uint8_t      Motorola         18        1         1         0         0         1         /     ||
         *   16      CDCU_MCUOverTemp_Err      uint8_t      Motorola         17        1         1         0         0         1         /     ||
         *   17      CDCU_MCUCom_Err           uint8_t      Motorola         16        1         1         0         0         1         /     ||
         *   18      CDCU_MCUMtrOverTemp_Err   uint8_t      Motorola         31        1         1         0         0         1         /     ||
         *   19      CDCU_MCUMtrPhase_Err      uint8_t      Motorola         30        1         1         0         0         1         /     ||
         *   20      CDCU_MCUEncoder_Err       uint8_t      Motorola         29        1         1         0         0         1         /     ||
         *=====================================================================================================================================*/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.throt_diag_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_DrvCmdPerd_Err", MOTOROLA, 7, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmdperd_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmdperd_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmdperd_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmdperd_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_DrvCmdDrop_Err", MOTOROLA, 6, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmddrop_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmddrop_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmddrop_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmddrop_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_DrvCmdChksum_Err", MOTOROLA, 5, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmdchksum_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmdchksum_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmdchksum_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmdchksum_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_DrvCmdOffline_Err", MOTOROLA, 4, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmdoffline_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmdoffline_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmdoffline_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmdoffline_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_DrvCmdOverRge_Err", MOTOROLA, 3, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmdoverrge_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmdoverrge_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmdoverrge_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_drvcmdoverrge_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCUMsgPerd_Err", MOTOROLA, 15, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumsgperd_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumsgperd_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumsgperd_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumsgperd_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCUMsgDrop_Err", MOTOROLA, 14, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumsgdrop_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumsgdrop_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumsgdrop_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumsgdrop_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCUMsgChksum_Err", MOTOROLA, 13, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumsgchksum_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumsgchksum_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumsgchksum_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumsgchksum_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCUMsgOffline_Err", MOTOROLA, 12, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumsgoffline_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumsgoffline_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumsgoffline_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumsgoffline_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        ////////////////////////////////////
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCUMainPwrCur_Err", MOTOROLA, 23, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumainpwrcur_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumainpwrcur_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumainpwrcur_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumainpwrcur_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCUCurtSensor_Err", MOTOROLA, 22, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcucurtsensor_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcucurtsensor_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcucurtsensor_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcucurtsensor_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCUPreChg_Err", MOTOROLA, 12, 21, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcuprechg_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcuprechg_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcuprechg_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcuprechg_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCUPwrOverVolt_Err", MOTOROLA, 20, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcupwrovervolt_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcupwrovervolt_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcupwrovervolt_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcupwrovervolt_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCUPwrUnderVolt_Err", MOTOROLA, 19, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcupwrundervolt_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcupwrundervolt_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcupwrundervolt_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcupwrundervolt_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCUPwrOverCurt_Err", MOTOROLA, 18, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcupwrovercurt_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcupwrovercurt_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcupwrovercurt_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcupwrovercurt_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCUOverTemp_Err", MOTOROLA, 17, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcuovertemp_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcuovertemp_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcuovertemp_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcuovertemp_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCUCom_Err", MOTOROLA, 16, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcucom_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcucom_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcucom_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcucom_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCUMtrOverTemp_Err", MOTOROLA, 31, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumtrovertemp_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumtrovertemp_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumtrovertemp_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumtrovertemp_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCUMtrPhase_Err", MOTOROLA, 30, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumtrphase_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumtrphase_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumtrphase_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcumtrphase_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCUEncoder_Err", MOTOROLA, 29, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcuencoder_err.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcuencoder_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcuencoder_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_diag_msg.cdcu_mcuencoder_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void ThrotDiag218::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.throt_diag_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.throt_diag_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
