/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "body_stat_0x219.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t BodyStat219::Init(void *const msg)
      {
        /** Message Definition==================================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName          |    @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_AcostOptic_WorkMode  uint8_t      Motorola         4         4         1         0         0         15        /     ||
         *   2       CDCU_HeadLamp_St        uint8_t      Motorola         15        1         1         0         0         1         /     ||
         *   3       CDCU_DblFlashLamp_St    uint8_t      Motorola         14        1         1         0         0         1         /     ||
         *   4       CDCU_TurnLLamp_St       uint8_t      Motorola         13        1         1         0         0         1         /     ||
         *   5       CDCU_TurnRLamp_St       uint8_t      Motorola         12        1         1         0         0         1         /     ||
         *   6       CDCU_BackLamp_St        uint8_t      Motorola         11        1         1         0         0         1         /     ||
         *   7       CDCU_Buzzer_St          uint8_t      Motorola         10        1         1         0         0         1         /     ||
         *   8       CDCU_Horn_St            uint8_t      Motorola         9         1         1         0         0         1         /     ||
         *   9       CDCU_RunLamp_St         uint8_t      Motorola         8         1         1         0         0         1         /     ||
         *   10      CDCU_BrkLamp_St         uint8_t      Motorola         23        1         1         0         0         1         /     ||
         *   11      CDCU_BCM_ErrLevel         uint8_t      Motorola         52        4         1         0         0         15        /     ||
         *====================================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.body_stat_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_AcostOptic_WorkMode", MOTOROLA, 4, 4, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_acostoptic_workmode.name,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_acostoptic_workmode.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_acostoptic_workmode.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_acostoptic_workmode.value, 0, 15, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_HeadLamp_St", MOTOROLA, 15, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_headlamp_stat.name,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_headlamp_stat.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_headlamp_stat.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_headlamp_stat.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_DblFlashLamp_St", MOTOROLA, 14, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_dblflashlamp_stat.name,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_dblflashlamp_stat.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_dblflashlamp_stat.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_dblflashlamp_stat.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_TurnLLamp_St", MOTOROLA, 13, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_turnllamp_stat.name,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_turnllamp_stat.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_turnllamp_stat.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_turnllamp_stat.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_TurnRLamp_St", MOTOROLA, 12, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_turnrlamp_stat.name,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_turnrlamp_stat.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_turnrlamp_stat.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_turnrlamp_stat.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BackLamp_St", MOTOROLA, 11, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_backlamp_stat.name,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_backlamp_stat.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_backlamp_stat.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_backlamp_stat.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_Buzzer_St", MOTOROLA, 10, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_buzzer_stat.name,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_buzzer_stat.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_buzzer_stat.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_buzzer_stat.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_Horn_St", MOTOROLA, 9, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_horn_stat.name,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_horn_stat.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_horn_stat.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_horn_stat.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_RunLamp_St", MOTOROLA, 8, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_runlamp_stat.name,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_runlamp_stat.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_runlamp_stat.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_runlamp_stat.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BrkLamp_St", MOTOROLA, 23, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_brklamp_stat.name,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_brklamp_stat.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_brklamp_stat.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_brklamp_stat.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BCM_ErrLevel", MOTOROLA, 52, 4, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_bcm_errlevel.name,
                                                           ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_bcm_errlevel.unit,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_bcm_errlevel.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.body_stat_msg.cdcu_bcm_errlevel.value, 0, 15, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void BodyStat219::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.body_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.body_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
