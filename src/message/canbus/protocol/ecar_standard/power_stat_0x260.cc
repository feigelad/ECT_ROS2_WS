/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "power_stat_0x260.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t PowerStat260::Init(void *const msg)
      {
        /** Message Definition====================================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName            |   @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_VehPwrSupply_St       uint8_t      Motorola         7         1         1         0         0         1         /     ||
         *   2       CDCU_ChasPwrSupply_St      uint8_t      Motorola         6         1         1         0         0         1         /     ||
         *   3       CDCU_12V1PwrSupply_St      uint8_t      Motorola         5         1         1         0         0         1         /     ||
         *   4       CDCU_12V2PwrSupply_St      uint8_t      Motorola         4         1         1         0         0         1         /     ||
         *   5       CDCU_AutDrv24VPwrSupply_St uint8_t      Motorola         3         1         1         0         0         1         /     ||
         *=====================================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.power_stat_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_VehPwrSupply_St", MOTOROLA, 7, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_vehpwrsupply_st.name,
                                                           ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_vehpwrsupply_st.unit,
                                                           &ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_vehpwrsupply_st.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_vehpwrsupply_st.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_ChasPwrSupply_St", MOTOROLA, 6, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_chaspwrsupply_st.name,
                                                           ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_chaspwrsupply_st.unit,
                                                           &ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_chaspwrsupply_st.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_chaspwrsupply_st.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_12V1PwrSupply_St", MOTOROLA, 5, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_12v1pwrsupply_st.name,
                                                           ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_12v1pwrsupply_st.unit,
                                                           &ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_12v1pwrsupply_st.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_12v1pwrsupply_st.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_12V2PwrSupply_St", MOTOROLA, 4, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_12v2pwrsupply_st.name,
                                                           ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_12v2pwrsupply_st.unit,
                                                           &ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_12v2pwrsupply_st.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_12v2pwrsupply_st.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_AutDrv24VPwrSupply_St", MOTOROLA, 3, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_autdrv24vpwrsupply_st.name,
                                                           ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_autdrv24vpwrsupply_st.unit,
                                                           &ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_autdrv24vpwrsupply_st.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.power_stat_msg.cdcu_autdrv24vpwrsupply_st.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void PowerStat260::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.power_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.power_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
