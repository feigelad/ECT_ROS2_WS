#include "ecar_motion.h"
#include "message/msg/ecar_chassis_stat.hpp"
#include "../../../components/motor/Imotor.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace status
    {
      bool EcarMotion::Init() {}

      float EcarMotion::VehicleSpeedMps(void *const stat)
      {
        ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
        return (ecar_stat->veh_dyna_stat_msg.cdcu_veh_longtdnalspd.value / 3.6);
      }

      int EcarMotion::VehicleRunDirect(void *const stat)
      {
        ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
        return ecar_stat->veh_dyna_stat_msg.cdcu_veh_rundir.value == components::Forward    ? 1
               : ecar_stat->veh_dyna_stat_msg.cdcu_veh_rundir.value == components::Backward ? -1
                                                                                            : 0;
      }

      bool EcarMotion::WheelSpeedFrontLeftValid(void *const stat)
      {
        ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
        return ecar_stat->front_wheel_spd_stat_msg.cdcu_veh_lfwhlspdvalid.value != 0 ? true : false;
      }

      float EcarMotion::WheelSpeedFrontLeftMps(void *const stat)
      {
        ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
        return ecar_stat->front_wheel_spd_stat_msg.cdcu_veh_lfwhlspd.value / 3.6;
      }

      bool EcarMotion::WheelSpeedFrontRightValid(void *const stat)
      {
        ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
        return ecar_stat->front_wheel_spd_stat_msg.cdcu_veh_rtwhlspdvalid.value != 0 ? true : false;
      }

      float EcarMotion::WheelSpeedFrontRightMps(void *const stat)
      {
        ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
        return ecar_stat->front_wheel_spd_stat_msg.cdcu_veh_rtwhlspd.value / 3.6;
      }

      bool EcarMotion::WheelSpeedBackLeftValid(void *const stat)
      {
        ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
        return ecar_stat->rear_wheel_spd_stat_msg.cdcu_veh_lfwhlspdvalid.value != 0 ? true : false;
      }

      float EcarMotion::WheelSpeedBackLeftMps(void *const stat)
      {
        ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
        return ecar_stat->rear_wheel_spd_stat_msg.cdcu_veh_lfwhlspd.value / 3.6;
      }

      bool EcarMotion::WheelSpeedBackRightValid(void *const stat)
      {
        ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
        return ecar_stat->rear_wheel_spd_stat_msg.cdcu_veh_rtwhlspdvalid.value != 0 ? true : false;
      }

      float EcarMotion::WheelSpeedBackRightMps(void *const stat)
      {
        ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
        return ecar_stat->rear_wheel_spd_stat_msg.cdcu_veh_rtwhlspd.value / 3.6;
      }

      float EcarMotion::LonAccelMps2(void *const stat)
      {
        ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
        return ecar_stat->imu_stat_msg.ins570d_acc_stat_msg.imu_accelx.value * 9.8;
      }
      float EcarMotion::YawRateRadps(void *const stat)
      {
        // ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
        // return ecar_stat->imu_stat_msg.
      }
    }
  }
}