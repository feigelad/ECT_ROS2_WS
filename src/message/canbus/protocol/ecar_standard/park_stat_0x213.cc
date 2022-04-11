/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "park_stat_0x213.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t ParkStat213::Init(void *const msg)
      {
        /** Message Definition=========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName    |   @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_Park_WorkMode uint8_t      Motorola         4         4         1         0         0         15         /     ||
         *   2       CDCU_Park_St       uint8_t      Motorola         2         2         1         0         0         3         /     ||
         *   4       CDCU_Park_ErrLevel uint8_t      Motorola         52        4         1         0         0         15         /     ||
         *===========================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.park_stat_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_Park_WorkMode", MOTOROLA, 4, 4, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.park_stat_msg.cdcu_park_workmode.name,
                                                           ecar_chassis_ptr_->stat_msg.park_stat_msg.cdcu_park_workmode.unit,
                                                           &ecar_chassis_ptr_->stat_msg.park_stat_msg.cdcu_park_workmode.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.park_stat_msg.cdcu_park_workmode.value, 0, 15, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_Park_St", MOTOROLA, 2, 2, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.park_stat_msg.cdcu_park_st.name,
                                                           ecar_chassis_ptr_->stat_msg.park_stat_msg.cdcu_park_st.unit,
                                                           &ecar_chassis_ptr_->stat_msg.park_stat_msg.cdcu_park_st.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.park_stat_msg.cdcu_park_st.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_Park_ErrLevel", MOTOROLA, 52, 4, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.park_stat_msg.cdcu_park_errlevel.name,
                                                           ecar_chassis_ptr_->stat_msg.park_stat_msg.cdcu_park_errlevel.unit,
                                                           &ecar_chassis_ptr_->stat_msg.park_stat_msg.cdcu_park_errlevel.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.park_stat_msg.cdcu_park_errlevel.value, 0, 15, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void ParkStat213::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.park_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.park_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
