/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "veh_stat_0x240.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t VehStat240::Init(void *const msg)
      {
        /** Message Definition==================================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName          |   @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_Veh_RunMode         uint8_t      Motorola         4         4         1         0         0         15        /     ||
         *   2       CDCU_Veh_ErrLevel        uint8_t      Motorola         0         2         1         0         0         3         /     ||
         *   3       CDCU_VehCrashContact_St  uint8_t      Motorola         15        1         1         0         0         1         /     ||
         *   4       CDCU_VehCrashTrg_St      uint8_t      Motorola         14        1         1         0         0         1         /     ||
         *   5       CDCU_VehEmgcySwh_St      uint8_t      Motorola         13        1         1         0         0         1         /     ||
         *   6       CDCU_LvBat_LowVolt       uint8_t      Motorola         12        1         1         0         0         1         /     ||
         *   7       CDCU_Bat_LowSOC          uint8_t      Motorola         11        1         1         0         0         1         /     ||
         *   8       CDCU_BMS_ErrLevel        uint8_t      Motorola         9         2         1         0         0         3         /     ||
         *   9       CDCU_MCU_ErrLevel        uint8_t      Motorola         22        2         1         0         0         3         /     ||
         *   10      CDCU_EHB_ErrLevel        uint8_t      Motorola         20        2         1         0         0         3         /     ||
         *   11      CDCU_EPS_ErrLevel        uint8_t      Motorola         18        2         1         0         0         3         /     ||
         *   12      CDCU_EPB_ErrLevel        uint8_t      Motorola         16        2         1         0         0         3         /     ||
         *===================================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.veh_stat_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_Veh_RunMode", MOTOROLA, 4, 4, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_veh_runmode.name,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_veh_runmode.unit,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_veh_runmode.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_veh_runmode.value, 0, 15, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_Veh_ErrLevel", MOTOROLA, 0, 2, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_veh_errlevel.name,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_veh_errlevel.unit,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_veh_errlevel.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_veh_errlevel.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_VehCrashContact_St", MOTOROLA, 15, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_vehcrashcontact_st.name,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_vehcrashcontact_st.unit,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_vehcrashcontact_st.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_vehcrashcontact_st.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_VehCrashTrg_St", MOTOROLA, 14, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_vehcrashtrg_st.name,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_vehcrashtrg_st.unit,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_vehcrashtrg_st.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_vehcrashtrg_st.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_VehEmgcySwh_St", MOTOROLA, 13, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_vehemgcyswh_st.name,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_vehemgcyswh_st.unit,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_vehemgcyswh_st.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_vehemgcyswh_st.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_LvBat_LowVolt", MOTOROLA, 12, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_lvbat_lowvolt.name,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_lvbat_lowvolt.unit,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_lvbat_lowvolt.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_lvbat_lowvolt.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_Bat_LowSOC", MOTOROLA, 11, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_bat_lowsoc.name,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_bat_lowsoc.unit,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_bat_lowsoc.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_bat_lowsoc.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_BMS_ErrLevel", MOTOROLA, 9, 2, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_bms_errlevel.name,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_bms_errlevel.unit,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_bms_errlevel.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_bms_errlevel.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_MCU_ErrLevel", MOTOROLA, 22, 2, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_mcu_errlevel.name,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_mcu_errlevel.unit,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_mcu_errlevel.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_mcu_errlevel.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EHB_ErrLevel", MOTOROLA, 20, 2, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_ehb_errlevel.name,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_ehb_errlevel.unit,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_ehb_errlevel.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_ehb_errlevel.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EPS_ErrLevel", MOTOROLA, 18, 2, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_eps_errlevel.name,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_eps_errlevel.unit,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_eps_errlevel.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_eps_errlevel.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EPB_ErrLevel", MOTOROLA, 16, 2, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_epb_errlevel.name,
                                                           ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_epb_errlevel.unit,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_epb_errlevel.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.veh_stat_msg.cdcu_epb_errlevel.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void VehStat240::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x240 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.veh_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.veh_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
