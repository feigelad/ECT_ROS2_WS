#include "magnetic.h"
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
      namespace park
      {
        bool Magnetic::Init() { return true; }

        bool Magnetic::RealParkStat(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->park_stat_msg.cdcu_park_st.value != 0 ? true : false;
        }

        uint8_t Magnetic::WorkMode(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->park_stat_msg.cdcu_park_workmode.value;
        }

        uint8_t Magnetic::FaultLevel(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->park_stat_msg.cdcu_park_errlevel.value;
        }

        void Magnetic::AutoPark(double stamp, bool target, void *const cmd)
        {
          ::message::msg::EcarChassisCmd *ecar_cmd = (::message::msg::EcarChassisCmd *)cmd;
          ecar_cmd->park_cmd_msg.adcu_prk_enable.time_stamp = stamp;
          ecar_cmd->park_cmd_msg.adcu_prk_enable.value = target ? 1 : 0;
        }

        void Magnetic::CldPark(double stamp, bool target, void *const cmd)
        {
          ::message::msg::EcarChassisCmd *ecar_cmd = (::message::msg::EcarChassisCmd *)cmd;
          ecar_cmd->cloud_drive_cmd_msg.adcu_cld_prkenable.time_stamp = stamp;
          ecar_cmd->cloud_drive_cmd_msg.adcu_cld_prkenable.value = target ? 1 : 0;
        }
      }
    }
  }
}