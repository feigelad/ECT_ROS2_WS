/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "veh_dyna_stat_0x250.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t VehDynaStat250::Init(void *const msg)
      {
        /** Message Definition==================================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName          |  @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_Veh_LongtdnalSpd   float        Motorola        24         16       1/256       0         0       256      kmph    ||
         *   2       CDCU_Veh_LongtdnalAccSpd float        Motorola        8          16       0.01       -40       -40      615.35   mps^2   ||
         *   3       CDCU_Veh_Curvture       float        Motorola        40         16       0.001      -3        -3       3.5535    /      ||
         *   4       CDCU_Veh_RunDir         uint8_t      Motorola        54         2          1         0         0       3         /      ||
         *====================================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.veh_dyna_stat_msg.header.frame_id = Name();
        if (false == AddSignal<float>(CanSignal<float>("CDCU_Veh_LongtdnalSpd", MOTOROLA, 24, 16, 1 / 256.0, 0,
                                                       ecar_chassis_ptr_->stat_msg.veh_dyna_stat_msg.cdcu_veh_longtdnalspd.name,
                                                       ecar_chassis_ptr_->stat_msg.veh_dyna_stat_msg.cdcu_veh_longtdnalspd.unit,
                                                       &ecar_chassis_ptr_->stat_msg.veh_dyna_stat_msg.cdcu_veh_longtdnalspd.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.veh_dyna_stat_msg.cdcu_veh_longtdnalspd.value, 0, 256, "kmph")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_Veh_LongtdnalAccSpd", MOTOROLA, 8, 16, 0.01, -40,
                                                       ecar_chassis_ptr_->stat_msg.veh_dyna_stat_msg.cdcu_veh_longtdnalacc.name,
                                                       ecar_chassis_ptr_->stat_msg.veh_dyna_stat_msg.cdcu_veh_longtdnalacc.unit,
                                                       &ecar_chassis_ptr_->stat_msg.veh_dyna_stat_msg.cdcu_veh_longtdnalacc.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.veh_dyna_stat_msg.cdcu_veh_longtdnalacc.value, -40, 615.35, "mps^2")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_Veh_Curvture", MOTOROLA, 40, 16, 0.001, -3,
                                                       ecar_chassis_ptr_->stat_msg.veh_dyna_stat_msg.cdcu_veh_curvture.name,
                                                       ecar_chassis_ptr_->stat_msg.veh_dyna_stat_msg.cdcu_veh_curvture.unit,
                                                       &ecar_chassis_ptr_->stat_msg.veh_dyna_stat_msg.cdcu_veh_curvture.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.veh_dyna_stat_msg.cdcu_veh_curvture.value, -3, 3.5535, "1/m")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_Veh_RunDir", MOTOROLA, 54, 2, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.veh_dyna_stat_msg.cdcu_veh_rundir.name,
                                                           ecar_chassis_ptr_->stat_msg.veh_dyna_stat_msg.cdcu_veh_rundir.unit,
                                                           &ecar_chassis_ptr_->stat_msg.veh_dyna_stat_msg.cdcu_veh_rundir.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.veh_dyna_stat_msg.cdcu_veh_rundir.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void VehDynaStat250::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.veh_dyna_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.veh_dyna_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
