/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "brake_diag_0x212.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t BrakeDiag212::Init(void *const msg)
      {
        /** Message Definition,  LiFei, 2020/7/7 ============================================================================================\
         *  @SN |   @SignalName     |      @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_BrkCmdPerd_Err    uint8_t      Motorola         7         1         1         0         0         1         /     ||
         *   2       CDCU_BrkCmdDrop_Err    uint8_t      Motorola         6         1         1         0         0         1         /     ||
         *   3       CDCU_BrkCmdChksum_Err  uint8_t      Motorola         5         1         1         0         0         1         /     ||
         *   4       CDCU_BrkCmdOffline_Err uint8_t      Motorola         4         1         1         0         0         1         /     ||
         *   5       CDCU_BrkCmdOverRge_Err uint8_t      Motorola         3         1         1         0         0         1         /     ||
         *   6       CDCU_EHBMsgPerd_Err    uint8_t      Motorola         15        1         1         0         0         1         /     ||
         *   7       CDCU_EHBMsgDrop_Err    uint8_t      Motorola         14        1         1         0         0         1         /     ||
         *   8       CDCU_EHBMsgChksum_Err  uint8_t      Motorola         13        1         1         0         0         1         /     ||
         *   9       CDCU_EHBMsgOffline_Err uint8_t      Motorola         12        1         1         0         0         1         /     ||
         *   10      CDCU_BrkFldLack_Err    uint8_t      Motorola         23        1         1         0         0         1         /     ||
         *   11      CDCU_EHBCurtSenor_Err  uint8_t      Motorola         22        1         1         0         0         1         /     ||
         *   12      CDCU_EHBTempSenor_Err  uint8_t      Motorola         21        1         1         0         0         1         /     ||
         *   13      CDCU_EHBOverTemp_Err   uint8_t      Motorola         20        1         1         0         0         1         /     ||
         *   14      CDCU_BrkPwrSupply_Err  uint8_t      Motorola         19        1         1         0         0         1         /     ||
         *   15      CDCU_BrkPresrSenor_Err uint8_t      Motorola         18        1         1         0         0         1         /     ||
         *   16      CDCU_EHBPwrSupply_Err  uint8_t      Motorola         17        1         1         0         0         1         /     ||
         *   17      CDCU_ECUPwrSupply_Err  uint8_t      Motorola         16        1         1         0         0         1         /     ||
         *   18      CDCU_DrvBridge_Err     uint8_t      Motorola         30        1         1         0         0         1         /     ||
         *   19      CDCU_DrvMtr_Err        uint8_t      Motorola         31        1         1         0         0         1         /     ||
         *   20      CDCU_EHBCAN_Err        uint8_t      Motorola         29        1         1         0         0         1         /     ||
         *==================================================================================================================================*/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.brake_diag_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BrkCmdPerd_Err", MOTOROLA, 7, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmdperd_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmdperd_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmdperd_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmdperd_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BrkCmdDrop_Err", MOTOROLA, 6, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmddrop_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmddrop_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmddrop_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmddrop_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BrkCmdChksum_Err", MOTOROLA, 5, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmdchksum_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmdchksum_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmdchksum_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmdchksum_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BrkCmdOffline_Err", MOTOROLA, 4, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmdoffline_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmdoffline_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmdoffline_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmdoffline_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BrkCmdOverRge_Err", MOTOROLA, 3, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmdoverrge_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmdoverrge_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmdoverrge_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkcmdoverrge_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EHBMsgPerd_Err", MOTOROLA, 15, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbmsgperd_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbmsgperd_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbmsgperd_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbmsgperd_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EHBMsgDrop_Err", MOTOROLA, 14, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbmsgdrop_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbmsgdrop_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbmsgdrop_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbmsgdrop_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EHBMsgChksum_Err", MOTOROLA, 13, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbmsgchksum_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbmsgchksum_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbmsgchksum_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbmsgchksum_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EHBMsgOffline_Err", MOTOROLA, 12, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbmsgoffline_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbmsgoffline_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbmsgoffline_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbmsgoffline_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BrkFldLack_Err", MOTOROLA, 23, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkfldlack_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkfldlack_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkfldlack_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkfldlack_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EHBCurtSenor_Err", MOTOROLA, 22, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbcurtsenor_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbcurtsenor_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbcurtsenor_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbcurtsenor_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EHBTempSenor_Err", MOTOROLA, 21, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbtempsenor_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbtempsenor_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbtempsenor_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbtempsenor_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EHBOverTemp_Err", MOTOROLA, 20, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbovertemp_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbovertemp_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbovertemp_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbovertemp_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BrkPwrSupply_Err", MOTOROLA, 19, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkpwrsupply_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkpwrsupply_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkpwrsupply_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkpwrsupply_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BrkPresrSenor_Err", MOTOROLA, 18, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkpresrsenor_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkpresrsenor_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkpresrsenor_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_brkpresrsenor_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EHBPwrSupply_Err", MOTOROLA, 17, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbpwrsupply_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbpwrsupply_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbpwrsupply_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbpwrsupply_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_ECUPwrSupply_Err", MOTOROLA, 16, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ecupwrsupply_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ecupwrsupply_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ecupwrsupply_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ecupwrsupply_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_DrvBridge_Err", MOTOROLA, 31, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_drvbridge_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_drvbridge_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_drvbridge_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_drvbridge_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_DrvMtr_Err", MOTOROLA, 30, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_drvmtr_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_drvmtr_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_drvmtr_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_drvmtr_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EHBCAN_Err", MOTOROLA, 29, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbcan_err.name,
                                                           ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbcan_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbcan_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.brake_diag_msg.cdcu_ehbcan_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void BrakeDiag212::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.brake_diag_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.brake_diag_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
