/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "steer_stat_0x215.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t SteerStat215::Init(void *const msg)
      {
        /** Message Definition=========================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName      |    @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_EPS_WorkMode     uint8_t      Motorola         4         4         1         0         0        15         /     ||
         *   2       CDCU_EPS_StrTrq       float        Motorola         8         8        0.1      -12.8     -12.8      12.7      Nm     ||
         *   3       CDCU_EPS_StrWhlAngle  float        Motorola         24        16       0.1      -1024     -500       500       deg    ||
         *   4       CDCU_EPS_WhlSpd       float        Motorola         40        16        1       -10000    -10000     10000     deg/s  ||
         *   5       CDCU_EPS_ErrLevel     uint8_t      Motorola         52        4         1         0         0        15         /     ||
         *===========================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.steer_stat_msg.header.frame_id = Name();
        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EPS_WorkMode", MOTOROLA, 4, 4, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_workmode.name,
                                                           ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_workmode.unit,
                                                           &ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_workmode.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_workmode.value, 0, 15, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_EPS_StrTrq", MOTOROLA, 8, 8, 0.1, -12.8,
                                                       ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_strtrq.name,
                                                       ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_strtrq.unit,
                                                       &ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_strtrq.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_strtrq.value, -12.8, 12.7, "Nm")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_EPS_StrWhlAngle", MOTOROLA, 24, 16, 0.005, -90,
                                                       ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_strwhlangle.name,
                                                       ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_strwhlangle.unit,
                                                       &ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_strwhlangle.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_strwhlangle.value, -90, 90, "deg")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<float>(CanSignal<float>("CDCU_EPS_WhlSpd", MOTOROLA, 40, 16, 0.01, -180,
                                                       ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_whlspd.name,
                                                       ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_whlspd.unit,
                                                       &ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_whlspd.time_stamp,
                                                       &ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_whlspd.value, -180, 180, "deg/s")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EPS_ErrLevel", MOTOROLA, 52, 4, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_errlevel.name,
                                                           ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_errlevel.unit,
                                                           &ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_errlevel.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.steer_stat_msg.cdcu_eps_errlevel.value, 0, 15, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void SteerStat215::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.steer_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.steer_stat_msg.header.stamp = clock.now();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
