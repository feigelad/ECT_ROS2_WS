/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "throt_stat_0x217.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t ThrotStat217::Init(void *const msg)
      {
        /** Message Definition===========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName    |   @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_MCU_WorkMode  uint8_t      Motorola         4         4         1         0         0         15        /     ||
         *   2       CDCU_MCU_GearAct   uint8_t      Motorola         2         2         1         0         0         3         /     ||
         *   3       CDCU_MCU_RunDir    uint8_t      Motorola         0         2         1         0         0         3         /     ||
         *   4       CDCU_MCU_ThrotAct  float        Motorola         8         8        0.4        0         0        100        %     ||
         *   5       CDCU_MCU_MtrCurt   float        Motorola         24        16       0.1        0         0       6553.5     kmph   ||
         *   6       CDCU_MCU_MtrSpd    float        Motorola         40        16        1         0         0       10000      rpm    ||
         *   7       CDCU_MCU_ErrLevel  uint8_t      Motorola         52         4        1         0         0         3         /     ||
         *=============================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.throt_stat_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCU_WorkMode", MOTOROLA, 4, 4, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_workmode.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_workmode.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_workmode.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_workmode.value, 0, 15, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCU_GearAct", MOTOROLA, 2, 2, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_gearact.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_gearact.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_gearact.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_gearact.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCU_RunDir", MOTOROLA, 0, 2, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_rundir.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_rundir.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_rundir.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_rundir.value, 0, 2, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_MCU_ThrotAct", MOTOROLA, 8, 8, 0.4, 0,
                                                       ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_throtact.name,
                                                       ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_throtact.unit,
                                                       &ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_throtact.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_throtact.value, 0, 100, "%")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_MCU_MtrCurt", MOTOROLA, 24, 16, 0.1, 0,
                                                       ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_mtrcurt.name,
                                                       ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_mtrcurt.unit,
                                                       &ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_mtrcurt.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_mtrcurt.value, 0, 6552.5, "A")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_MCU_MtrSpd", MOTOROLA, 40, 16, 1, 0,
                                                       ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_mtrspd.name,
                                                       ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_mtrspd.unit,
                                                       &ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_mtrspd.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_mtrspd.value, 0, 10000, "rpm")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCU_ErrLevel", MOTOROLA, 52, 4, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_errlevel.name,
                                                           ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_errlevel.unit,
                                                           &ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_errlevel.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.throt_stat_msg.cdcu_mcu_errlevel.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void ThrotStat217::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.throt_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.throt_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
