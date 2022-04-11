#include "ceps.h"
#include <vector>
#include <math.h>

#include "message/msg/ecar_chassis_stat.hpp"
#include "message/msg/ecar_chassis_cmd.hpp"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace steer
      {
        bool CEPS::Init() { return true; }

        float CEPS::RealStrAngelRad(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->steer_stat_msg.cdcu_eps_strwhlangle.value * M_PI / 180.0;
        }
        uint8_t CEPS::WorkMode(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->steer_stat_msg.cdcu_eps_workmode.value;
        }
        uint8_t CEPS::FaultLevel(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->steer_stat_msg.cdcu_eps_errlevel.value;
        }
        void CEPS::AutoSteer(double stamp, enum SteerMode steer_mode, float target_val, void *const cmd)
        {
          ::message::msg::EcarChassisCmd *ecar_cmd = (::message::msg::EcarChassisCmd *)cmd;
          if (steer_mode == Str_Angle)
          {
            ecar_cmd->steer_cmd_msg.adcu_str_ctrlmode.time_stamp = stamp;
            ecar_cmd->steer_cmd_msg.adcu_str_ctrlmode.value = static_cast<uint8_t>(steer_mode);
            ecar_cmd->steer_cmd_msg.adcu_str_tgtangle.time_stamp = stamp;
            ecar_cmd->steer_cmd_msg.adcu_str_tgtangle.value = target_val;
          }
          else
          {
            ecar_cmd->steer_cmd_msg.adcu_str_ctrlmode.time_stamp = stamp;
            ecar_cmd->steer_cmd_msg.adcu_str_ctrlmode.value = static_cast<uint8_t>(Str_Angle);
            ecar_cmd->steer_cmd_msg.adcu_str_tgtangle.time_stamp = stamp;
            ecar_cmd->steer_cmd_msg.adcu_str_tgtangle.value = 0;
          }
          ecar_cmd->steer_cmd_msg.adcu_str_tgtanglespd.time_stamp = stamp;
          ecar_cmd->steer_cmd_msg.adcu_str_tgtanglespd.value = 50;
        }

        void CEPS::CldSteer(double stamp, enum SteerMode steer_mode, float target_val, void *const cmd)
        {
          ::message::msg::EcarChassisCmd *ecar_cmd = (::message::msg::EcarChassisCmd *)cmd;
          if (steer_mode == components::steer::Str_Angle)
          {
            ecar_cmd->cloud_drive_cmd_msg.adcu_cld_tgtstrangle.time_stamp = stamp;
            ecar_cmd->cloud_drive_cmd_msg.adcu_cld_tgtstrangle.value = target_val;
          }
          else
          {
            ecar_cmd->cloud_drive_cmd_msg.adcu_cld_tgtstrangle.time_stamp = stamp;
            ecar_cmd->cloud_drive_cmd_msg.adcu_cld_tgtstrangle.value = 0;
          }
        }
      }
    }
  }
}