/*
 * @Descripttion: 
 * @version: 
 * @Author: LiFei
 * @Date: 2020-02-18 01:10:11
 * @LastEditors: LiFei
 * @LastEditTime: 2020-02-18 12:17:03
 */
#include <rclcpp/rclcpp.hpp>
#include "anticrsh_stat_0x221.h"
#include <numeric>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      canbus_error_t AnticrshStat221::Init(void *const msg)
      {
        /** Message Definition================================================================================================================
         *  LiFei, 2020/7/7
         *  @SN |   @SignalName         |   @DataType  | @ByteOrder | @StartBit | @BitLen | @Factor | @Offset | @MinVal | @MaxVal | @Unit    ||
         *   1       CDCU_FtCrashContact_St  uint8_t      Motorola         7         1         1         0         0         1         /     ||
         *   2       CDCU_RrCrashContact_St  uint8_t      Motorola         6         1         1         0         0         1         /     ||
         *   3       CDCU_FtCrashTrg_St      uint8_t      Motorola         5         1         1         0         0         1         /     ||
         *   4       CDCU_RrCrashTrg_St      uint8_t      Motorola         4         1         1         0         0         1         /     ||
         *   5       CDCU_EmgcyPark_St       uint8_t      Motorola         0         2         1         0         0         3         /     ||
         *==================================================================================================================================**/
        ecar_chassis_ptr_ = (::message::msg::EcarChassis *)msg;
        ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.header.frame_id = Name();

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_FtCrashContact_St", MOTOROLA, 7, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_ftcrashcontact_st.name,
                                                           ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_ftcrashcontact_st.unit,
                                                           &ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_ftcrashcontact_st.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_ftcrashcontact_st.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_RrCrashContact_St", MOTOROLA, 6, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_rrcrashcontact_st.name,
                                                           ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_rrcrashcontact_st.unit,
                                                           &ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_rrcrashcontact_st.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_rrcrashcontact_st.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_FtCrashTrg_St", MOTOROLA, 5, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_ftcrashtrg_st.name,
                                                           ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_ftcrashtrg_st.unit,
                                                           &ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_ftcrashtrg_st.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_ftcrashtrg_st.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_RrCrashTrg_St", MOTOROLA, 4, 1, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_rrcrashtrg_st.name,
                                                           ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_rrcrashtrg_st.unit,
                                                           &ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_rrcrashtrg_st.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_rrcrashtrg_st.value, 0, 1, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }

        if (false == AddSignal<uint8_t>(CanSignal<uint8_t>("CDCU_EmgcyPark_St", MOTOROLA, 0, 2, 1, 0,
                                                           ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_emgcypark_st.name,
                                                           ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_emgcypark_st.unit,
                                                           &ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_emgcypark_st.time_stamp,
                                                           &ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.cdcu_emgcypark_st.value, 0, 3, "/")))
        {
          //TODO: Add signal fault, LiFei, 2020/7/7
          is_inited_.exchange(false);
          return CAN_ERROR_PROTOCOL_INIT_FAILED(ID());
        }
        is_inited_.exchange(true);
        return CAN_RESULT_OK;
      }

      void AnticrshStat221::ParseMsg(const uint8_t *data, uint8_t len, double stamp)
      {

        if (data == nullptr || is_inited_.load() == false)
        {
          //TODO: todo what is not decided, LiFei, 2020/7/9
          return;
        }
        // std::cout << "0x217 received !" << std::endl;
        Parse(data, len, stamp);
        // ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.header.seq++;
        rclcpp::Clock clock;
        ecar_chassis_ptr_->stat_msg.anticrsh_stat_msg.header.stamp = clock.now();;
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana
