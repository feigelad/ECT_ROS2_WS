/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "veh_info_0x2a0.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t VehInfo2a0::Init(void *const msg)
      {
        /** Message Definition==================================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName          |   @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal |    @Unit   ||
         *   1       CDCU_Veh_VIN             uint32_t     Motorola         24        32        1         0         0     4294967295      /     ||
         *   2       CDCU_HW_Version          float        Motorola         32        8        0.1        0         0       25.5          /     ||
         *   3       CDCU_SW_Version          float        Motorola         52        12       0.1        0         0       409.5         /     ||
         *===================================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.veh_info_msg.header.frame_id = Name();
        if (false == AddSignal<uint32_t>(CanSignal<uint32_t>("CDCU_Veh_VIN", MOTOROLA, 24, 32, 1, 0,
                                                             ecar_chassis_ptr_->stat_msg.veh_info_msg.cdcu_veh_vin.name,
                                                             ecar_chassis_ptr_->stat_msg.veh_info_msg.cdcu_veh_vin.unit,
                                                             &ecar_chassis_ptr_->stat_msg.veh_info_msg.cdcu_veh_vin.time_stamp,
                                                             &ecar_chassis_ptr_->stat_msg.veh_info_msg.cdcu_veh_vin.value, 0, 4294967295, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_HW_Version", MOTOROLA, 32, 8, 0.1, 0,
                                                       ecar_chassis_ptr_->stat_msg.veh_info_msg.cdcu_hw_version.name,
                                                       ecar_chassis_ptr_->stat_msg.veh_info_msg.cdcu_hw_version.unit,
                                                       &ecar_chassis_ptr_->stat_msg.veh_info_msg.cdcu_hw_version.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.veh_info_msg.cdcu_hw_version.value, 0, 25.5, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_SW_Version", MOTOROLA, 52, 12, 0.1, 0,
                                                       ecar_chassis_ptr_->stat_msg.veh_info_msg.cdcu_sw_version.name,
                                                       ecar_chassis_ptr_->stat_msg.veh_info_msg.cdcu_sw_version.unit,
                                                       &ecar_chassis_ptr_->stat_msg.veh_info_msg.cdcu_sw_version.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.veh_info_msg.cdcu_sw_version.value, 0, 409.5, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void VehInfo2a0::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.veh_info_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.veh_info_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
