/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "total_trip_stat_0x254.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t TotalTripStat254::Init(void *const msg)
      {
        /** Message Definition=========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName    |      @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_TotalTrip_Meter  float        Motorola         4         4         1         0         0         15         /     ||
         *   2       CDCU_TotalODO_Meter   float        Motorola         2         2         1         0         0         3         /     ||
         *===========================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.total_trip_msg.header.frame_id = Name();
        if (false == AddSignal<float>(CanSignal<float>("CDCU_TotalTrip_Meter", MOTOROLA, 8, 16, 0.1, 0,
                                                       ecar_chassis_ptr_->stat_msg.total_trip_msg.cdcu_trip_meter.name,
                                                       ecar_chassis_ptr_->stat_msg.total_trip_msg.cdcu_trip_meter.unit,
                                                       &ecar_chassis_ptr_->stat_msg.total_trip_msg.cdcu_trip_meter.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.total_trip_msg.cdcu_trip_meter.value, 0, 6553.5, "km")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_TotalODO_Meter", MOTOROLA, 40, 24, 0.1, 0,
                                                       ecar_chassis_ptr_->stat_msg.total_trip_msg.cdcu_odo_meter.name,
                                                       ecar_chassis_ptr_->stat_msg.total_trip_msg.cdcu_odo_meter.unit,
                                                       &ecar_chassis_ptr_->stat_msg.total_trip_msg.cdcu_odo_meter.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.total_trip_msg.cdcu_odo_meter.value, 0, 1677721.5, "km")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void TotalTripStat254::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.total_trip_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.total_trip_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
