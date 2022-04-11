#include "motor_dbstar_2p2kw.h"
#include <vector>
#include <math.h>
#include <algorithm>

#include "message/msg/ecar_chassis_stat.hpp"
#include "message/msg/ecar_chassis_cmd.hpp"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace motor
      {
        bool MotorDBStar2p2kw::Init(float k_m_throt, float k_m_brake, float mspd_fb_cutoff, const std::map<int, float> &p_m_map)
        {
          Km_throt_.exchange(k_m_throt);
          Km_brake_.exchange(k_m_brake);
          mspd_fd_cutoff_.exchange(mspd_fb_cutoff);
          Pm_map_ = p_m_map;
          return true;
        }

        float MotorDBStar2p2kw::GetThrotKm() const
        {
          return Km_throt_.load();
        }

        float MotorDBStar2p2kw::GetBrakeKm() const
        {
          return Km_brake_.load();
        }

        float MotorDBStar2p2kw::GetPm(float throt)
        {
          std::vector<int> throt_vec;
          std::map<int, float> pm_map;
          {
            std::unique_lock<std::mutex> lck(mutex_);
            pm_map = Pm_map_;
          }
          if (pm_map.empty())
            return 0;

          for (auto itr = pm_map.begin(); itr != pm_map.end(); itr++)
            throt_vec.push_back(itr->first);

          int left = 0;
          int right = throt_vec.size() - 1;
          if (throt <= throt_vec[left])
            return pm_map[throt_vec[left]];

          if (throt >= throt_vec[right])
            return pm_map[throt_vec[right]];

          while (left + 1 < right)
          {
            int x = (left + right) / 2;
            if (throt == throt_vec[x])
              return pm_map[throt_vec[x]];
            else if (throt > throt_vec[x])
              left = x;
            else if (throt < throt_vec[x])
              right = x;
            else
            {
            }
          }

          return pm_map[throt_vec[left]] + (pm_map[throt_vec[right]] - pm_map[throt_vec[left]]) * (throt - static_cast<float>(throt_vec[left])) / static_cast<float>(throt_vec[right] - throt_vec[left]);
        }

        float MotorDBStar2p2kw::DirectModel(float throttle, int gear, float mspd_rpm, int real_direct)
        {
          int k_direct = real_direct == Backward  ? 1
                         : real_direct == Forward ? -1
                                                  : 0;
          int k_gear = gear == Gear_D   ? 1
                       : gear == Gear_R ? -1
                                        : 0;

          float k_rpm2radps = 2 * M_PI / 60.0;
          float Tm = 0;
          float P_kw = 0;
          if (throttle * fabs(k_gear) < 0 || k_direct * k_gear < 0) //Braking process
          {
            Tm = -1 * k_direct * Km_brake_.load() * fabs(throttle);
            if (mspd_rpm < mspd_fd_cutoff_.load())
              Tm = Tm * mspd_rpm / mspd_fd_cutoff_.load();
          }
          else if (throttle >= 0) //Throttle process
            Tm = k_gear * Km_throt_.load() * fabs(throttle);
          else
            Tm = 0;
          P_kw = Tm * mspd_rpm * k_direct * k_rpm2radps * 0.001;

          if (P_kw < 0)
            Tm = fabs(P_kw) <= fabs(GetPm(-1 * fabs(throttle))) ? Tm : -1 * k_direct * fabs(GetPm(-1 * fabs(throttle))) / (mspd_rpm * k_rpm2radps * 0.001);
          else
            Tm = P_kw <= GetPm(fabs(throttle)) ? Tm : k_gear * GetPm(fabs(throttle)) / (mspd_rpm * k_rpm2radps * 0.001);

          return Tm;
        }

        float MotorDBStar2p2kw::InverseModel(float tgt_torq, int real_gear, float mspd_rpm, int real_direct)
        {
          int k_direct = real_direct == Backward  ? 1
                         : real_direct == Forward ? -1
                                                  : 0;
          int k_gear = real_gear == Gear_D   ? 1
                       : real_gear == Gear_R ? -1
                                             : 0;

          float k_rpm2radps = 2 * M_PI / 60.0;
          float Tm = 0;
          float P_kw = 0;
          float tgt_throt = 0;
          if (tgt_torq * k_direct < 0 && fabs(k_gear) > 0) //Braking process
          {
            tgt_throt = std::max<float>(-100, -1 * fabs(tgt_torq) / Km_brake_.load());
            if (mspd_rpm < mspd_fd_cutoff_.load() && mspd_rpm > 50)
              tgt_throt = std::max<float>(-100, -1 * fabs(tgt_torq) * mspd_fd_cutoff_.load() / (mspd_rpm * Km_brake_.load()));
            P_kw = tgt_torq * mspd_rpm * k_direct * k_rpm2radps * 0.001;
          }
          else if (tgt_torq * k_gear > 0) //Throttle process
          {
            tgt_throt = std::min<float>(100, fabs(tgt_torq) / Km_throt_.load());
            P_kw = tgt_torq * mspd_rpm * k_direct * 0.001;
          }
          else
          {
            tgt_throt = 0;
            P_kw = 0;
          }

          if (P_kw < 0)
          {
            if (P_kw < GetPm(tgt_throt))
            {
              if (P_kw <= GetPm(-100))
                tgt_throt = -100;
              else
              {
                for (int i = 0; - 100 + i * 5 < tgt_throt; i++)
                {
                  float left = -100 + i * 5;
                  float right = -100 + (i + 1) * 5;
                  if (P_kw >= GetPm(left) && P_kw < GetPm(right))
                  {
                    tgt_throt = left + (right - left) * (P_kw - GetPm(left)) / (GetPm(right) - GetPm(left));
                  }
                }
              }
            }
          }
          else
          {
            if (P_kw > GetPm(tgt_throt))
            {
              if (P_kw > GetPm(100))
                tgt_throt = 100;
              else
              {
                for (int i = 0; 100 - i * 5 > tgt_throt; i++)
                {
                  float left = 100 - (i + 1) * 5;
                  float right = 100 - i * 5;
                  if (P_kw > GetPm(left) && P_kw <= GetPm(right))
                  {
                    tgt_throt = left + (right - left) * (P_kw - GetPm(left)) / (GetPm(right) - GetPm(left));
                  }
                }
              }
            }
          }

          return tgt_throt;
        }

        void MotorDBStar2p2kw::TorqRangeAvailable(float &torq_max, float &torq_min, float mspd_rpm, int real_direct)
        {
        }

        uint8_t MotorDBStar2p2kw::WorkMode(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->throt_stat_msg.cdcu_mcu_workmode.value;
        }

        uint8_t MotorDBStar2p2kw::FaultLevel(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->throt_stat_msg.cdcu_mcu_errlevel.value;
        }

        float MotorDBStar2p2kw::RealThrottle(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->throt_stat_msg.cdcu_mcu_throtact.value;
        }

        float MotorDBStar2p2kw::RealCurrentA(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->throt_stat_msg.cdcu_mcu_mtrcurt.value;
        }

        int MotorDBStar2p2kw::RealGear(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->throt_stat_msg.cdcu_mcu_gearact.value == Gear_D   ? 1
                 : ecar_stat->throt_stat_msg.cdcu_mcu_gearact.value == Gear_R ? -1
                                                                              : 0;
        }

        int MotorDBStar2p2kw::RealDirect(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->throt_stat_msg.cdcu_mcu_rundir.value == Forward    ? 1
                 : ecar_stat->throt_stat_msg.cdcu_mcu_rundir.value == Backward ? -1
                                                                               : 0;
        }

        float MotorDBStar2p2kw::RealMspdRpm(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          return ecar_stat->throt_stat_msg.cdcu_mcu_mtrspd.value;
        }

        void MotorDBStar2p2kw::AutoDrive(double stamp, enum AccelMode acc_mode, int gear, float drive_value, void *const cmd)
        {
          ::message::msg::EcarChassisCmd *ecar_cmd = (::message::msg::EcarChassisCmd *)cmd;
          //throt
          ecar_cmd->throt_cmd_msg.adcu_drv_ctrlmode.time_stamp = stamp;
          ecar_cmd->throt_cmd_msg.adcu_drv_ctrlmode.value = static_cast<uint8_t>(acc_mode);
          ecar_cmd->throt_cmd_msg.adcu_drv_tgtgear.time_stamp = stamp;
          uint8_t act_gear = gear < -0.5  ? 2
                             : gear > 0.5 ? 1
                                          : 0;
          ecar_cmd->throt_cmd_msg.adcu_drv_tgtgear.value = act_gear;
          if (acc_mode == Acc_Throttle)
          {
            ecar_cmd->throt_cmd_msg.adcu_drv_tgtpedpos.time_stamp = stamp;
            ecar_cmd->throt_cmd_msg.adcu_drv_tgtpedpos.value = drive_value;
            ecar_cmd->throt_cmd_msg.adcu_drv_tgtvehaccspd.time_stamp = stamp;
            ecar_cmd->throt_cmd_msg.adcu_drv_tgtvehaccspd.value = 0;
            ecar_cmd->throt_cmd_msg.adcu_drv_tgtvehspd.time_stamp = stamp;
            ecar_cmd->throt_cmd_msg.adcu_drv_tgtvehspd.value = 0;
          }
          else if (acc_mode == Acc_Speed)
          {
            ecar_cmd->throt_cmd_msg.adcu_drv_tgtpedpos.time_stamp = stamp;
            ecar_cmd->throt_cmd_msg.adcu_drv_tgtpedpos.value = 0;
            ecar_cmd->throt_cmd_msg.adcu_drv_tgtvehaccspd.time_stamp = stamp;
            ecar_cmd->throt_cmd_msg.adcu_drv_tgtvehaccspd.value = 0;
            ecar_cmd->throt_cmd_msg.adcu_drv_tgtvehspd.time_stamp = stamp;
            ecar_cmd->throt_cmd_msg.adcu_drv_tgtvehspd.value = drive_value;
          }
          else
          {
            ecar_cmd->throt_cmd_msg.adcu_drv_tgtpedpos.time_stamp = stamp;
            ecar_cmd->throt_cmd_msg.adcu_drv_tgtpedpos.value = 0;
            ecar_cmd->throt_cmd_msg.adcu_drv_tgtvehaccspd.time_stamp = stamp;
            ecar_cmd->throt_cmd_msg.adcu_drv_tgtvehaccspd.value = 0;
            ecar_cmd->throt_cmd_msg.adcu_drv_tgtvehspd.time_stamp = stamp;
            ecar_cmd->throt_cmd_msg.adcu_drv_tgtvehspd.value = 0;
          }
          ecar_cmd->throt_cmd_msg.adcu_drv_vehspdlimit.time_stamp = stamp;
          ecar_cmd->throt_cmd_msg.adcu_drv_vehspdlimit.value = 30;
        }

        void MotorDBStar2p2kw::CldDrive(double stamp, enum AccelMode acc_mode, int gear, float drive_value, void *const cmd)
        {
          ::message::msg::EcarChassisCmd *ecar_cmd = (::message::msg::EcarChassisCmd *)cmd;
          //throt
          ecar_cmd->cloud_drive_cmd_msg.adcu_cld_throtmode.time_stamp = stamp;
          ecar_cmd->cloud_drive_cmd_msg.adcu_cld_throtmode.value = static_cast<uint8_t>(acc_mode);
          uint8_t act_gear = gear < 0.5   ? 2
                             : gear > 0.5 ? 1
                                          : 0;
          ecar_cmd->cloud_drive_cmd_msg.adcu_cld_tgtgear.time_stamp = stamp;
          ecar_cmd->cloud_drive_cmd_msg.adcu_cld_tgtgear.value = act_gear;
          ecar_cmd->cloud_drive_cmd_msg.adcu_cld_tgtthrotval.time_stamp = stamp;
          ecar_cmd->cloud_drive_cmd_msg.adcu_cld_tgtthrotval.value = static_cast<float>(drive_value);
        }
      }
    }
  }
}