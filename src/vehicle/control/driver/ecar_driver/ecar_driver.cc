#include "ecar_driver.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace control
    {
      namespace driver
      {
        //Interfaces
        bool EcarDriver::Init(std::shared_ptr<common::log::ILog> logger,
                              std::shared_ptr<common::time::ITime> timer,
                              std::shared_ptr<components::motor::IMotor> motor,
                              std::shared_ptr<components::brake::IBrake> brake,
                              std::shared_ptr<components::steer::ISteer> steer,
                              std::shared_ptr<components::park::IPark> park,
                              std::shared_ptr<components::body::IBody> body)
        {
          if (logger == nullptr)
            return false;

          logger_ = logger;

          if (timer == nullptr)
          {
            logger_->Log(common::log::Log_Error, "EcarDriver init failed, timer is nullptr!");
            return false;
          }
          timer_ = timer;

          if (motor == nullptr)
          {
            logger_->Log(common::log::Log_Error, "EcarDriver init failed, motor is nullptr!");
            return false;
          }
          motor_ = motor;

          if (brake == nullptr)
          {
            logger_->Log(common::log::Log_Error, "EcarDriver init failed, brake is nullptr!");
            return false;
          }
          brake_ = brake;

          if (steer == nullptr)
          {
            logger_->Log(common::log::Log_Error, "EcarDriver init failed, steer is nullptr!");
            return false;
          }
          steer_ = steer;

          if (park == nullptr)
          {
            logger_->Log(common::log::Log_Error, "EcarDriver init failed, park is nullptr!");
            return false;
          }
          park_ = park;

          if (body == nullptr)
          {
            logger_->Log(common::log::Log_Error, "EcarDriver init failed, body is nullptr!");
            return false;
          }
          body_ = body;

          logger_->Log(common::log::Log_Info, "EcarDriver init successful !");
          return true;
        }

        void EcarDriver::Drive(enum DriveMode dm,
                               enum components::motor::AccelMode acc_mode, int gear, float acc_val,
                               enum components::brake::BrakeMode brk_mode, float brk_val,
                               enum components::steer::SteerMode str_mode, float str_val,
                               bool prk_enable, uint8_t body_ctrl,
                               const void *const stat, void *const cmd)
        {
          ::message::msg::EcarChassisStat *veh_stat = (::message::msg::EcarChassisStat *)stat;
          ::message::msg::EcarChassisCmd *ecar_cmd = (::message::msg::EcarChassisCmd *)cmd;
          //SetMode
          uint8_t auto_active = 0;
          uint8_t cld_active = 0;
          SetMode(auto_active, cld_active, dm, veh_stat);
          // logger_->Log(nirvana::common::log::Log_Info, "active is" + std::to_string(auto_active));
          //Drive
          //active
          double time_stamp = timer_->Now2Sec();
          AutoActive(auto_active, time_stamp, ecar_cmd);
          CldActive(cld_active, time_stamp, ecar_cmd);

          //Throt
          if (veh_stat->veh_stat_msg.cdcu_veh_runmode.value == DM_Automatic)
          {
            motor_->AutoDrive(time_stamp, acc_mode, gear, acc_val, cmd);
            brake_->AutoBrake(time_stamp, brk_mode, brk_val, cmd);
            steer_->AutoSteer(time_stamp, str_mode, str_val, cmd);
            park_->AutoPark(time_stamp, prk_enable, cmd);
            body_->AutoBodySwitch(time_stamp, body_ctrl, cmd);
          }
          if (veh_stat->veh_stat_msg.cdcu_veh_runmode.value == DM_Cloud)
          {
            motor_->CldDrive(time_stamp, acc_mode, gear, acc_val, ecar_cmd);
            brake_->CldBrake(time_stamp, brk_mode, brk_val, ecar_cmd);
            steer_->CldSteer(time_stamp, str_mode, str_val, ecar_cmd);
            park_->CldPark(time_stamp, prk_enable, ecar_cmd);
            body_->CldBodySwitch(time_stamp, body_ctrl, cmd);
          }
        }

        //Privates
        void EcarDriver::SetMode(uint8_t &auto_active, uint8_t &cld_active, enum DriveMode dm, ::message::msg::EcarChassisStat *veh_stat)
        {
          static uint8_t s_auto_active = 0;
          static uint8_t s_cld_active = 0;
          static uint8_t s_wait_count = 0;
          if (++s_wait_count > 50)
            s_wait_count = 0;
          if (veh_stat->veh_stat_msg.cdcu_veh_runmode.value == DM_Standby)
          {
            if (dm == DM_Automatic)
            {
              // if (s_wait_count == 0)
              //   s_auto_active ^= 1;
              s_auto_active = 1;
              s_cld_active = 0;
            }
            else if (dm == DM_Cloud)
            {
              if (s_wait_count == 0)
                s_cld_active ^= 1;
              s_auto_active = 0;
            }
            else
            {
              s_cld_active = 0;
              s_auto_active = 0;
            }
          }
          else if (veh_stat->veh_stat_msg.cdcu_veh_runmode.value == DM_Automatic)
          {
            if (dm == DM_Automatic)
            {
              s_auto_active = 1;
              s_cld_active = 0;
            }
            else if (dm == DM_Cloud)
            {
              if (s_wait_count == 0)
                s_cld_active ^= 1;
              s_auto_active = 0;
            }
            else
            {
              s_cld_active = 0;
              s_auto_active = 0;
            }
          }
          else if (veh_stat->veh_stat_msg.cdcu_veh_runmode.value == DM_Cloud)
          {
            if (dm == DM_Automatic)
            {
              s_auto_active = 0;
              s_cld_active = 0;
            }
            else if (dm == DM_Cloud)
            {
              s_auto_active = 0;
              s_cld_active = 1;
            }
            else
            {
              s_cld_active = 0;
              s_auto_active = 0;
            }
          }
          else
          {
            s_auto_active = 0;
            s_cld_active = 0;
          }

          auto_active = s_auto_active;
          cld_active = s_cld_active;
        }

        void EcarDriver::AutoActive(uint8_t active, double stamp, ::message::msg::EcarChassisCmd *const cmd)
        {
          cmd->throt_cmd_msg.adcu_drv_active.time_stamp = stamp;
          cmd->throt_cmd_msg.adcu_drv_active.value = active;
          cmd->brake_cmd_msg.adcu_brk_active.time_stamp = stamp;
          cmd->brake_cmd_msg.adcu_brk_active.value = active;
          cmd->steer_cmd_msg.adcu_str_active.time_stamp = stamp;
          cmd->steer_cmd_msg.adcu_str_active.value = active;
          cmd->park_cmd_msg.adcu_prk_active.time_stamp = stamp;
          cmd->park_cmd_msg.adcu_prk_active.value = active;
        }

        void EcarDriver::CldActive(uint8_t active, double stamp, ::message::msg::EcarChassisCmd *const cmd)
        {
          cmd->cloud_drive_cmd_msg.adcu_cld_ctrlative.time_stamp = stamp;
          cmd->cloud_drive_cmd_msg.adcu_cld_ctrlative.value = active;
        }

        void EcarDriver::ThrotAutoDrive(enum components::motor::AccelMode acc_mode, int gear, float acc_val, double stamp, ::message::msg::EcarChassisCmd *const cmd)
        {
          //throt
          cmd->throt_cmd_msg.adcu_drv_ctrlmode.time_stamp = stamp;
          cmd->throt_cmd_msg.adcu_drv_ctrlmode.value = static_cast<uint8_t>(acc_mode);
          cmd->throt_cmd_msg.adcu_drv_tgtgear.time_stamp = stamp;
          uint8_t act_gear = gear < 0.5   ? 2
                             : gear > 0.5 ? 1
                                          : 0;
          cmd->throt_cmd_msg.adcu_drv_tgtgear.value = act_gear;
          if (acc_mode == components::motor::Acc_Throttle)
          {
            cmd->throt_cmd_msg.adcu_drv_tgtpedpos.time_stamp = stamp;
            cmd->throt_cmd_msg.adcu_drv_tgtpedpos.value = acc_val;
            cmd->throt_cmd_msg.adcu_drv_tgtvehspd.time_stamp = stamp;
            cmd->throt_cmd_msg.adcu_drv_tgtvehspd.value = 0;
          }
          else if (acc_mode == components::motor::Acc_Accel)
          {
            cmd->throt_cmd_msg.adcu_drv_tgtpedpos.time_stamp = stamp;
            cmd->throt_cmd_msg.adcu_drv_tgtpedpos.value = 0;
            cmd->throt_cmd_msg.adcu_drv_tgtvehspd.time_stamp = stamp;
            cmd->throt_cmd_msg.adcu_drv_tgtvehspd.value = 0;
          }
          else if (acc_mode == components::motor::Acc_Speed)
          {
            cmd->throt_cmd_msg.adcu_drv_tgtpedpos.time_stamp = stamp;
            cmd->throt_cmd_msg.adcu_drv_tgtpedpos.value = 0;
            cmd->throt_cmd_msg.adcu_drv_tgtvehspd.time_stamp = stamp;
            cmd->throt_cmd_msg.adcu_drv_tgtvehspd.value = acc_val;
          }
          else
          {
            cmd->throt_cmd_msg.adcu_drv_tgtpedpos.time_stamp = stamp;
            cmd->throt_cmd_msg.adcu_drv_tgtpedpos.value = 0;
            cmd->throt_cmd_msg.adcu_drv_tgtvehspd.time_stamp = stamp;
            cmd->throt_cmd_msg.adcu_drv_tgtvehspd.value = 0;
          }
          cmd->throt_cmd_msg.adcu_drv_vehspdlimit.time_stamp = stamp;
          cmd->throt_cmd_msg.adcu_drv_vehspdlimit.value = 30;
        }

        void EcarDriver::ThrotCldDrive(enum components::motor::AccelMode acc_mode, int gear, float acc_val, double stamp, ::message::msg::EcarChassisCmd *const cmd)
        {
          cmd->cloud_drive_cmd_msg.adcu_cld_throtmode.time_stamp = stamp;
          cmd->cloud_drive_cmd_msg.adcu_cld_throtmode.value = static_cast<uint8_t>(acc_mode);
          uint8_t act_gear = gear < 0.5   ? 2
                             : gear > 0.5 ? 1
                                          : 0;
          cmd->cloud_drive_cmd_msg.adcu_cld_tgtgear.time_stamp = stamp;
          cmd->cloud_drive_cmd_msg.adcu_cld_tgtgear.value = act_gear;
          cmd->cloud_drive_cmd_msg.adcu_cld_tgtthrotval.time_stamp = stamp;
          cmd->cloud_drive_cmd_msg.adcu_cld_tgtthrotval.value = static_cast<uint8_t>(acc_val);
        }

        void EcarDriver::BrakeAutoDrive(enum components::brake::BrakeMode brk_mode, float brk_val, double stamp, ::message::msg::EcarChassisCmd *const cmd)
        {
          cmd->brake_cmd_msg.adcu_brk_ctrlmode.time_stamp = stamp;
          cmd->brake_cmd_msg.adcu_brk_ctrlmode.value = static_cast<uint8_t>(brk_mode);
          if (brk_mode == components::brake::Brk_Pedpos)
          {
            cmd->brake_cmd_msg.adcu_brk_tgtpedpos.time_stamp = stamp;
            cmd->brake_cmd_msg.adcu_brk_tgtpedpos.value = brk_val;
            cmd->brake_cmd_msg.adcu_brk_tgtpress.time_stamp = stamp;
            cmd->brake_cmd_msg.adcu_brk_tgtpress.value = 0;
            cmd->brake_cmd_msg.adcu_brk_tgtacc.time_stamp = stamp;
            cmd->brake_cmd_msg.adcu_brk_tgtacc.value = 0;
          }
          else if (brk_mode == components::brake::Brk_Press)
          {
            cmd->brake_cmd_msg.adcu_brk_tgtpedpos.time_stamp = stamp;
            cmd->brake_cmd_msg.adcu_brk_tgtpedpos.value = 0;
            cmd->brake_cmd_msg.adcu_brk_tgtpress.time_stamp = stamp;
            cmd->brake_cmd_msg.adcu_brk_tgtpress.value = brk_val;
            cmd->brake_cmd_msg.adcu_brk_tgtacc.time_stamp = stamp;
            cmd->brake_cmd_msg.adcu_brk_tgtacc.value = 0;
          }
          else if (brk_mode == components::brake::Brk_Deccel)
          {
            cmd->brake_cmd_msg.adcu_brk_tgtpedpos.time_stamp = stamp;
            cmd->brake_cmd_msg.adcu_brk_tgtpedpos.value = 0;
            cmd->brake_cmd_msg.adcu_brk_tgtpress.time_stamp = stamp;
            cmd->brake_cmd_msg.adcu_brk_tgtpress.value = 0;
            cmd->brake_cmd_msg.adcu_brk_tgtacc.time_stamp = stamp;
            cmd->brake_cmd_msg.adcu_brk_tgtacc.value = brk_val;
          }
          else if (brk_mode == components::brake::Brk_Speed)
          {
            cmd->brake_cmd_msg.adcu_brk_tgtpedpos.time_stamp = stamp;
            cmd->brake_cmd_msg.adcu_brk_tgtpedpos.value = 0;
            cmd->brake_cmd_msg.adcu_brk_tgtpress.time_stamp = stamp;
            cmd->brake_cmd_msg.adcu_brk_tgtpress.value = 0;
            cmd->brake_cmd_msg.adcu_brk_tgtacc.time_stamp = stamp;
            cmd->brake_cmd_msg.adcu_brk_tgtacc.value = 0;
          }
          else
          {
            cmd->brake_cmd_msg.adcu_brk_tgtpedpos.time_stamp = stamp;
            cmd->brake_cmd_msg.adcu_brk_tgtpedpos.value = 0;
            cmd->brake_cmd_msg.adcu_brk_tgtpress.time_stamp = stamp;
            cmd->brake_cmd_msg.adcu_brk_tgtpress.value = 0;
            cmd->brake_cmd_msg.adcu_brk_tgtacc.time_stamp = stamp;
            cmd->brake_cmd_msg.adcu_brk_tgtacc.value = 0;
          }
        }

        void EcarDriver::BrakeCldDrive(enum components::brake::BrakeMode brk_mode, float brk_val, double stamp, ::message::msg::EcarChassisCmd *const cmd)
        {
          if (brk_mode == components::brake::Brk_Pedpos)
          {
            cmd->cloud_drive_cmd_msg.adcu_cld_tgtbrkval.time_stamp = stamp;
            cmd->cloud_drive_cmd_msg.adcu_cld_tgtbrkval.value = brk_val;
          }
          else
          {
            cmd->cloud_drive_cmd_msg.adcu_cld_tgtbrkval.time_stamp = stamp;
            cmd->cloud_drive_cmd_msg.adcu_cld_tgtbrkval.value = 0;
          }
        }

        void EcarDriver::SteerAutoDrive(enum components::steer::SteerMode str_mode, float str_val, double stamp, ::message::msg::EcarChassisCmd *const cmd)
        {
          cmd->steer_cmd_msg.adcu_str_ctrlmode.time_stamp = stamp;
          cmd->steer_cmd_msg.adcu_str_ctrlmode.value = static_cast<uint8_t>(str_mode);
          if (str_mode == components::steer::Str_Angle)
          {
            cmd->steer_cmd_msg.adcu_str_tgtangle.time_stamp = stamp;
            cmd->steer_cmd_msg.adcu_str_tgtangle.value = str_val;
            cmd->steer_cmd_msg.adcu_str_tgtcurvature.time_stamp = stamp;
            cmd->steer_cmd_msg.adcu_str_tgtcurvature.value = 0;
          }
          else if (str_mode == components::steer::Str_Curvature)
          {
            cmd->steer_cmd_msg.adcu_str_tgtangle.time_stamp = stamp;
            cmd->steer_cmd_msg.adcu_str_tgtangle.value = 0;
            cmd->steer_cmd_msg.adcu_str_tgtcurvature.time_stamp = stamp;
            cmd->steer_cmd_msg.adcu_str_tgtcurvature.value = str_val;
          }
          else
          {
            cmd->steer_cmd_msg.adcu_str_tgtangle.time_stamp = stamp;
            cmd->steer_cmd_msg.adcu_str_tgtangle.value = 0;
            cmd->steer_cmd_msg.adcu_str_tgtcurvature.time_stamp = stamp;
            cmd->steer_cmd_msg.adcu_str_tgtcurvature.value = 0;
          }
        }

        void EcarDriver::SteerCldDrive(enum components::steer::SteerMode str_mode, float str_val, double stamp, ::message::msg::EcarChassisCmd *const cmd)
        {
          if (str_mode == components::steer::Str_Angle)
          {
            cmd->cloud_drive_cmd_msg.adcu_cld_tgtstrangle.time_stamp = stamp;
            cmd->cloud_drive_cmd_msg.adcu_cld_tgtstrangle.value = str_val;
          }
          else
          {
            cmd->cloud_drive_cmd_msg.adcu_cld_tgtstrangle.time_stamp = stamp;
            cmd->cloud_drive_cmd_msg.adcu_cld_tgtstrangle.value = 0;
          }
        }

        void EcarDriver::ParkAutoDrive(bool prk_enable, double stamp, ::message::msg::EcarChassisCmd *const cmd)
        {
          cmd->park_cmd_msg.adcu_prk_enable.time_stamp = stamp;
          cmd->park_cmd_msg.adcu_prk_enable.value = prk_enable;
        }

        void EcarDriver::ParkCldDrive(bool prk_enable, double stamp, ::message::msg::EcarChassisCmd *const cmd)
        {
          cmd->cloud_drive_cmd_msg.adcu_cld_prkenable.time_stamp = stamp;
          cmd->cloud_drive_cmd_msg.adcu_cld_prkenable.value = prk_enable ? 1 : 0;
        }

        void EcarDriver::BodyAutoDrive(uint8_t body_ctrl, double stamp, ::message::msg::EcarChassisCmd *const cmd)
        {
          cmd->body_cmd_msg.adcu_headlamp_cmd.time_stamp = stamp;
          cmd->body_cmd_msg.adcu_headlamp_cmd.value = (body_ctrl & components::body::HEAD_LIGHT_ON) != 0 ? 1 : 0;
          cmd->body_cmd_msg.adcu_turnllamp_cmd.time_stamp = stamp;
          cmd->body_cmd_msg.adcu_turnllamp_cmd.value = (body_ctrl & components::body::TURN_LEFT_LIGHT_ON) != 0 ? 1 : 0;
          cmd->body_cmd_msg.adcu_turnrlamp_cmd.time_stamp = stamp;
          cmd->body_cmd_msg.adcu_turnrlamp_cmd.value = (body_ctrl & components::body::TURN_RIGHT_LIGHT_ON) != 0 ? 1 : 0;
          cmd->body_cmd_msg.adcu_dblflashlamp_cmd.time_stamp = stamp;
          cmd->body_cmd_msg.adcu_dblflashlamp_cmd.value = (body_ctrl & components::body::DOUBLE_FLASH_LIGHT_ON) != 0 ? 1 : 0;
          cmd->body_cmd_msg.adcu_backlamp_cmd.time_stamp = stamp;
          cmd->body_cmd_msg.adcu_backlamp_cmd.value = (body_ctrl & components::body::BACK_LIGHT_ON) != 0 ? 1 : 0;
          cmd->body_cmd_msg.adcu_buzzer_cmd.time_stamp = stamp;
          cmd->body_cmd_msg.adcu_buzzer_cmd.value = (body_ctrl & components::body::BUZZER_ON) != 0 ? 1 : 0;
          cmd->body_cmd_msg.adcu_horn_cmd.time_stamp = stamp;
          cmd->body_cmd_msg.adcu_horn_cmd.value = (body_ctrl & components::body::HORN_ON) != 0 ? 1 : 0;
          cmd->body_cmd_msg.adcu_runlamp_cmd.time_stamp = stamp;
          cmd->body_cmd_msg.adcu_runlamp_cmd.value = (body_ctrl & components::body::RUN_LIGHT_ON) != 0 ? 1 : 0;
        }

        void EcarDriver::BodyCldDrive(uint8_t body_ctrl, double stamp, ::message::msg::EcarChassisCmd *const cmd)
        {
        }
      }

    } // namespace control
  }
} // namespace nirvana