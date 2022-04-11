#include "disc_brake.h"
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
      namespace brake
      {
        bool DiscBrake::Init(float K_press2torq, float max_press)
        {
          K_press2torq_.exchange(K_press2torq);
          max_press_.exchange(max_press);
          return true;
        }
        float DiscBrake::GetKpress2torq() const
        {
          return K_press2torq_.load();
        }
        float DiscBrake::DirectModelPress2Torq(float brk_press)
        {
          return K_press2torq_.load() * brk_press;
        }
        float DiscBrake::InverseModelTorq2Press(float target_torq)
        {
          if (target_torq == 0)
            return 0;

          if (K_press2torq_.load() < 0.1)
            return max_press_.load();

          return std::min<float>(max_press_.load(), target_torq / K_press2torq_.load());
        }

        float DiscBrake::RealBrkPressBar(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->brake_stat_msg.cdcu_ehb_brkpresur.value;
        }

        uint8_t DiscBrake::WorkMode(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->brake_stat_msg.cdcu_ehb_brkmode.value;
        }

        uint8_t DiscBrake::FaultLevel(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->brake_stat_msg.cdcu_ehb_errlevel.value;
        }

        void DiscBrake::AutoBrake(double stamp, enum BrakeMode brake_mode, float target_val, void *const cmd)
        {
          ::message::msg::EcarChassisCmd *ecar_cmd = (::message::msg::EcarChassisCmd *)cmd;
          ecar_cmd->brake_cmd_msg.adcu_brk_ctrlmode.time_stamp = stamp;
          ecar_cmd->brake_cmd_msg.adcu_brk_ctrlmode.value = static_cast<uint8_t>(brake_mode);
          if (brake_mode == Brk_Press)
          {
            ecar_cmd->brake_cmd_msg.adcu_brk_tgtpedpos.time_stamp = stamp;
            ecar_cmd->brake_cmd_msg.adcu_brk_tgtpedpos.value = 0;
            ecar_cmd->brake_cmd_msg.adcu_brk_tgtpress.time_stamp = stamp;
            ecar_cmd->brake_cmd_msg.adcu_brk_tgtpress.value = target_val;
            ecar_cmd->brake_cmd_msg.adcu_brk_tgtacc.time_stamp = stamp;
            ecar_cmd->brake_cmd_msg.adcu_brk_tgtacc.value = 0;
          }
          else
          {
            ecar_cmd->brake_cmd_msg.adcu_brk_tgtpedpos.time_stamp = stamp;
            ecar_cmd->brake_cmd_msg.adcu_brk_tgtpedpos.value = 0;
            ecar_cmd->brake_cmd_msg.adcu_brk_tgtpress.time_stamp = stamp;
            ecar_cmd->brake_cmd_msg.adcu_brk_tgtpress.value = 0;
            ecar_cmd->brake_cmd_msg.adcu_brk_tgtacc.time_stamp = stamp;
            ecar_cmd->brake_cmd_msg.adcu_brk_tgtacc.value = 0;
          }
        }

        void DiscBrake::CldBrake(double stamp, enum BrakeMode brake_mode, float target_val, void *const cmd)
        {
          ::message::msg::EcarChassisCmd *ecar_cmd = (::message::msg::EcarChassisCmd *)cmd;
          if (brake_mode == components::brake::Brk_Pedpos)
          {
            ecar_cmd->cloud_drive_cmd_msg.adcu_cld_tgtbrkval.time_stamp = stamp;
            ecar_cmd->cloud_drive_cmd_msg.adcu_cld_tgtbrkval.value = target_val;
          }
          else
          {
            ecar_cmd->cloud_drive_cmd_msg.adcu_cld_tgtbrkval.time_stamp = stamp;
            ecar_cmd->cloud_drive_cmd_msg.adcu_cld_tgtbrkval.value = 0;
          }
        }
      }
    }
  }
}