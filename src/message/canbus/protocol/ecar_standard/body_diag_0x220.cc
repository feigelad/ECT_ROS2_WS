/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "body_diag_0x220.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t BodyDiag220::Init(void *const msg)
      {
        /** Message Definition=================================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName          |      @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_BodyCmdPerd_Err        uint8_t      Motorola         7         1         1         0         0         1         /     ||
         *   2       CDCU_BodyCmdDrop_Err        uint8_t      Motorola         6         1         1         0         0         1         /     ||
         *   3       CDCU_BodyCmdChksum_Err      uint8_t      Motorola         5         1         1         0         0         1         /     ||
         *   4       CDCU_BodyCmdOffline_Err     uint8_t      Motorola         4         1         1         0         0         1         /     ||
         *   5       CDCU_HeadLampOpenCirct_Err  uint8_t      Motorola         15        1         1         0         0         1         /     ||
         *   6       CDCU_HeadLampShrtCirct_Err  uint8_t      Motorola         14        1         1         0         0         1         /     ||
         *   7       CDCU_TurnLLampOpenCirct_Err uint8_t      Motorola         13        1         1         0         0         1         /     ||
         *   8       CDCU_TurnLLampShrtCirct_Err uint8_t      Motorola         12        1         1         0         0         1         /     ||
         *   9       CDCU_TurnRLampOpenCirct_Err uint8_t      Motorola         11        1         1         0         0         1         /     ||
         *   10      CDCU_TurnRLampShrtCirct_Err uint8_t      Motorola         10        1         1         0         0         1         /     ||
         *   11      CDCU_BackLampOpenCirct_Err  uint8_t      Motorola         9         1         1         0         0         1         /     ||
         *   12      CDCU_BackLampShrtCirct_Err  uint8_t      Motorola         8         1         1         0         0         1         /     ||
         *   13      CDCU_BuzzerOpenCirct_Err    uint8_t      Motorola         23        1         1         0         0         1         /     ||
         *   14      CDCU_BuzzerShrtCirct_Err    uint8_t      Motorola         22        1         1         0         0         1         /     ||
         *   15      CDCU_HornOpenCirct_Err      uint8_t      Motorola         21        1         1         0         0         1         /     ||
         *   16      CDCU_HornShrtCirct_Err      uint8_t      Motorola         20        1         1         0         0         1         /     ||
         *   17      CDCU_RunLampOpenCirct_Err   uint8_t      Motorola         19        1         1         0         0         1         /     ||
         *   18      CDCU_RunLampShrtCirct_Err   uint8_t      Motorola         18        1         1         0         0         1         /     ||
         *   19      CDCU_BrkLampOpenCirct_Err   uint8_t      Motorola         17        1         1         0         0         1         /     ||
         *   20      CDCU_BrkLampShrtCirct_Err   uint8_t      Motorola         16        1         1         0         0         1         /     ||
         *===================================================================================================================================*/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.body_diag_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BodyCmdPerd_Err", MOTOROLA, 7, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_bodycmdperd_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_bodycmdperd_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_bodycmdperd_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_bodycmdperd_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BodyCmdDrop_Err", MOTOROLA, 6, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_bodycmddrop_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_bodycmddrop_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_bodycmddrop_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_bodycmddrop_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BodyCmdChksum_Err", MOTOROLA, 5, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_bodycmdchksum_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_bodycmdchksum_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_bodycmdchksum_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_bodycmdchksum_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BodyCmdOffline_Err", MOTOROLA, 4, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_bodycmdoffline_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_bodycmdoffline_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_bodycmdoffline_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_bodycmdoffline_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_HeadLampOpenCirct_Err", MOTOROLA, 15, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_headlampopencirct_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_headlampopencirct_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_headlampopencirct_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_headlampopencirct_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_HeadLampShrtCirct_Err", MOTOROLA, 14, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_headlampshrtcirct_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_headlampshrtcirct_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_headlampshrtcirct_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_headlampshrtcirct_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_TurnLLampOpenCirct_Err", MOTOROLA, 13, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_turnllampopencirct_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_turnllampopencirct_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_turnllampopencirct_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_turnllampopencirct_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_TurnLLampShrtCirct_Err", MOTOROLA, 12, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_turnllampshrtcirct_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_turnllampshrtcirct_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_turnllampshrtcirct_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_turnllampshrtcirct_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_TurnRLampOpenCirct_Err", MOTOROLA, 11, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_turnrlampopencirct_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_turnrlampopencirct_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_turnrlampopencirct_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_turnrlampopencirct_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_TurnRLampShrtCirct_Err", MOTOROLA, 10, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_turnrlampshrtcirct_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_turnrlampshrtcirct_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_turnrlampshrtcirct_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_turnrlampshrtcirct_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BackLampOpenCirct_Err", MOTOROLA, 9, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_backlampopencirct_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_backlampopencirct_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_backlampopencirct_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_backlampopencirct_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BackLampShrtCirct_Err", MOTOROLA, 8, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_backlampshrtcirct_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_backlampshrtcirct_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_backlampshrtcirct_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_backlampshrtcirct_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BuzzerOpenCirct_Err", MOTOROLA, 23, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_buzzeropencirct_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_buzzeropencirct_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_buzzeropencirct_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_buzzeropencirct_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BuzzerShrtCirct_Err", MOTOROLA, 22, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_buzzershrtcirct_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_buzzershrtcirct_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_buzzershrtcirct_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_buzzershrtcirct_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_HornOpenCirct_Err", MOTOROLA, 21, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_hornopencirct_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_hornopencirct_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_hornopencirct_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_hornopencirct_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_HornShrtCirct_Err", MOTOROLA, 20, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_hornshrtcirct_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_hornshrtcirct_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_hornshrtcirct_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_hornshrtcirct_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_RunLampOpenCirct_Err", MOTOROLA, 19, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_runlampopencirct_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_runlampopencirct_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_runlampopencirct_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_runlampopencirct_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_RunLampShrtCirct_Err", MOTOROLA, 18, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_runlampshrtcirct_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_runlampshrtcirct_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_runlampshrtcirct_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_runlampshrtcirct_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BrkLampOpenCirct_Err", MOTOROLA, 17, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_brklampopencirct_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_brklampopencirct_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_brklampopencirct_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_brklampopencirct_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BrkLampShrtCirct_Err", MOTOROLA, 16, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_brklampshrtcirct_err.name,
                                                           ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_brklampshrtcirct_err.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_brklampshrtcirct_err.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_diag_msg.cdcu_brklampshrtcirct_err.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void BodyDiag220::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {
        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.body_diag_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.body_diag_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
