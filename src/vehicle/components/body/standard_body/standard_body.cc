#include "standard_body.h"
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
      namespace body
      {
        bool StandardBody::Init() { return true; }

        uint8_t StandardBody::WorkMode(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->body_stat_msg.cdcu_acostoptic_workmode.value;
        }

        uint8_t StandardBody::FaultLevel(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->body_stat_msg.cdcu_bcm_errlevel.value;
        }

        bool StandardBody::HeadLampOn(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->body_stat_msg.cdcu_headlamp_stat.value != 0 ? true : false;          
        }

        bool StandardBody::BackLampOn(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->body_stat_msg.cdcu_backlamp_stat.value != 0 ? true : false;          
        }

        bool StandardBody::LeftSteerLampOn(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->body_stat_msg.cdcu_turnllamp_stat.value != 0 ? true : false;          
        }

        bool StandardBody::RightSteerLampOn(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->body_stat_msg.cdcu_turnrlamp_stat.value != 0 ? true : false;          
        }

        bool StandardBody::HarzardOn(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->body_stat_msg.cdcu_dblflashlamp_stat.value != 0 ? true : false;          
        }

        bool StandardBody::BrakeLampOn(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->body_stat_msg.cdcu_brklamp_stat.value != 0 ? true : false;
          
        }

        bool StandardBody::KlaxonOn(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->body_stat_msg.cdcu_horn_stat.value != 0 ? true : false;          
        }

        void StandardBody::AutoBodySwitch(double stamp, uint8_t body_ctrl, void *const cmd)
        {
          ::message::msg::EcarChassisCmd *ecar_cmd = (::message::msg::EcarChassisCmd *)cmd;
          ecar_cmd->body_cmd_msg.adcu_headlamp_cmd.time_stamp = stamp;
          ecar_cmd->body_cmd_msg.adcu_headlamp_cmd.value = (body_ctrl & HEAD_LIGHT_ON) != 0 ? 1 : 0;
          ecar_cmd->body_cmd_msg.adcu_turnllamp_cmd.time_stamp = stamp;
          ecar_cmd->body_cmd_msg.adcu_turnllamp_cmd.value = (body_ctrl & TURN_LEFT_LIGHT_ON) != 0 ? 1 : 0;
          ecar_cmd->body_cmd_msg.adcu_turnrlamp_cmd.time_stamp = stamp;
          ecar_cmd->body_cmd_msg.adcu_turnrlamp_cmd.value = (body_ctrl & TURN_RIGHT_LIGHT_ON) != 0 ? 1 : 0;
          ecar_cmd->body_cmd_msg.adcu_dblflashlamp_cmd.time_stamp = stamp;
          ecar_cmd->body_cmd_msg.adcu_dblflashlamp_cmd.value = (body_ctrl & DOUBLE_FLASH_LIGHT_ON) != 0 ? 1 : 0;
          ecar_cmd->body_cmd_msg.adcu_backlamp_cmd.time_stamp = stamp;
          ecar_cmd->body_cmd_msg.adcu_backlamp_cmd.value = (body_ctrl & BACK_LIGHT_ON) != 0 ? 1 : 0;
          ecar_cmd->body_cmd_msg.adcu_buzzer_cmd.time_stamp = stamp;
          ecar_cmd->body_cmd_msg.adcu_buzzer_cmd.value = (body_ctrl & BUZZER_ON) != 0 ? 1 : 0;
          ecar_cmd->body_cmd_msg.adcu_horn_cmd.time_stamp = stamp;
          ecar_cmd->body_cmd_msg.adcu_horn_cmd.value = (body_ctrl & HORN_ON) != 0 ? 1 : 0;
          ecar_cmd->body_cmd_msg.adcu_runlamp_cmd.time_stamp = stamp;
          ecar_cmd->body_cmd_msg.adcu_runlamp_cmd.value = (body_ctrl & RUN_LIGHT_ON) != 0 ? 1 : 0;
        }

        void StandardBody::CldBodySwitch(double stamp, uint8_t body_ctrl, void *const cmd)
        {
          ::message::msg::EcarChassisCmd *ecar_cmd = (::message::msg::EcarChassisCmd *)cmd;
          ecar_cmd->cloud_body_cmd_msg.adcu_cld_headlampcmd.time_stamp = stamp;
          ecar_cmd->cloud_body_cmd_msg.adcu_cld_headlampcmd.value = (body_ctrl & HEAD_LIGHT_ON) != 0 ? 1 : 0;
          ecar_cmd->cloud_body_cmd_msg.adcu_cld_turnllampcmd.time_stamp = stamp;
          ecar_cmd->cloud_body_cmd_msg.adcu_cld_turnllampcmd.value = (body_ctrl & TURN_LEFT_LIGHT_ON) != 0 ? 1 : 0;
          ecar_cmd->cloud_body_cmd_msg.adcu_cld_turnrlampcmd.time_stamp = stamp;
          ecar_cmd->cloud_body_cmd_msg.adcu_cld_turnrlampcmd.value = (body_ctrl & TURN_RIGHT_LIGHT_ON) != 0 ? 1 : 0;
          ecar_cmd->cloud_body_cmd_msg.adcu_cld_dblflashlampcmd.time_stamp = stamp;
          ecar_cmd->cloud_body_cmd_msg.adcu_cld_dblflashlampcmd.value = (body_ctrl & DOUBLE_FLASH_LIGHT_ON) != 0 ? 1 : 0;
          ecar_cmd->cloud_body_cmd_msg.adcu_cld_backlampcmd.time_stamp = stamp;
          ecar_cmd->cloud_body_cmd_msg.adcu_cld_backlampcmd.value = (body_ctrl & BACK_LIGHT_ON) != 0 ? 1 : 0;
          ecar_cmd->cloud_body_cmd_msg.adcu_cld_buzzercmd.time_stamp = stamp;
          ecar_cmd->cloud_body_cmd_msg.adcu_cld_buzzercmd.value = (body_ctrl & BUZZER_ON) != 0 ? 1 : 0;
          ecar_cmd->cloud_body_cmd_msg.adcu_cld_horncmd.time_stamp = stamp;
          ecar_cmd->cloud_body_cmd_msg.adcu_cld_horncmd.value = (body_ctrl & HORN_ON) != 0 ? 1 : 0;
          ecar_cmd->cloud_body_cmd_msg.adcu_cld_runlampcmd.time_stamp = stamp;
          ecar_cmd->cloud_body_cmd_msg.adcu_cld_runlampcmd.value = (body_ctrl & RUN_LIGHT_ON) != 0 ? 1 : 0;
        }
      }
    }
  }
}