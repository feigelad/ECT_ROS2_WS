#include "monorail_model.h"
#include <math.h>

#include "message/msg/ecar_chassis_stat.hpp"

namespace nirvana
{
  namespace vehicle
  {
    namespace dynamic
    {
      //Interfaces
      bool MonorailModel::Init(const std::shared_ptr<components::motor::IMotor> &motor,
                               const std::shared_ptr<components::gearbox::IGearBox> &gearbox,
                               const std::shared_ptr<components::reducer::IReducer> &reducer,
                               const std::shared_ptr<components::wheel::IWheel> &front_left_wheel,
                               const std::shared_ptr<components::wheel::IWheel> &front_right_wheel,
                               const std::shared_ptr<components::wheel::IWheel> &back_left_wheel,
                               const std::shared_ptr<components::wheel::IWheel> &back_right_wheel,
                               const std::shared_ptr<components::brake::IBrake> &front_left_brake,
                               const std::shared_ptr<components::brake::IBrake> &front_right_brake,
                               const std::shared_ptr<components::brake::IBrake> &back_left_brake,
                               const std::shared_ptr<components::brake::IBrake> &back_right_brake,
                               const DynaParams &params)
      {
        if (motor == nullptr)
          return false;
        motor_ = motor;

        if (gearbox == nullptr)
          return false;
        gearbox_ = gearbox;

        if (reducer == nullptr)
          return false;
        reducer_ = reducer;

        if (front_left_wheel == nullptr)
          return false;
        front_left_wheel_ = front_left_wheel;

        if (front_right_wheel == nullptr)
          return false;
        front_right_wheel_ = front_right_wheel;

        if (back_left_wheel == nullptr)
          return false;
        back_left_wheel_ = back_left_wheel;

        if (back_right_wheel == nullptr)
          return false;
        back_right_wheel_ = back_right_wheel;

        if (front_left_brake == nullptr)
          return false;
        front_left_brake_ = front_left_brake;

        if (front_right_brake == nullptr)
          return false;
        front_right_brake_ = front_right_brake;

        if (back_left_brake == nullptr)
          return false;
        back_left_brake_ = back_left_brake;

        if (back_right_brake == nullptr)
          return false;
        back_right_brake_ = back_right_brake;

        vehdyna_params_ = params;

        return true;
      }

      float MonorailModel::LonDynamic(float throttle, int gear, float brake, float steer_rad, float pitch_rad, float mspd_rpm, float vspd_mps, int direct)
      {
        DynaParams params;
        {
          std::unique_lock<std::mutex> lck(mutex_);
          params = vehdyna_params_;
        }

        int k_direct = direct == components::Forward    ? 1
                       : direct == components::Backward ? -1
                                                        : 0;

        float Z = Throttle2Z(throttle, gear, mspd_rpm, direct);
        float Fst = Pitch2Fst(pitch_rad);
        float Fr = Direct2Fr(direct);
        float Flx = Vspd2Flx(vspd_mps, direct);

        float BabsAvailable = Press2BAvailable(brake);

        if (direct == components::Stopped)
        {
          float a_without_brake = LonAcceleration(Z, 0, Fst, Fr, Flx, steer_rad);
          if (a_without_brake > 0)
          {
            float B = -1 * BabsAvailable;
            return std::max<float>(0, LonAcceleration(Z, B, Fst, Fr, Flx, steer_rad));
          }
          else if (a_without_brake < 0)
          {
            float B = BabsAvailable;
            return std::min<float>(0, LonAcceleration(Z, B, Fst, Fr, Flx, steer_rad));
          }
          else
            return 0;
        }
        else
        {
          float B = -1 * direct * BabsAvailable;
          return LonAcceleration(Z, B, Fst, Fr, Flx, steer_rad);
        }
      }

      float MonorailModel::LatDynamic(float steer_rad, float vspd_mps)
      {
        return LatCurvature(steer_rad);
      }

      void MonorailModel::LonDynamicInverse(float &throttle, float &brake, float target_accel, int gear, float steer_rad, float pitch_rad, float mspd_rpm, float vspd_mps, int direct)
      {
        float Fst = Pitch2Fst(pitch_rad);
        float Fr = Direct2Fr(direct);
        float Flx = Vspd2Flx(vspd_mps, direct);
        float force_needed = LonAcceleration2ForceNeeded(target_accel, Fst, Fr, Flx, steer_rad);
        bool stop_to_go = false;
        if (direct == components::Stopped)
        {
          if (fabs(target_accel) < 0.01) //brake
            stop_to_go = false;
          else //throttle
            stop_to_go = true;
        }
        ForceNeeded2Act(throttle, brake, force_needed, stop_to_go, gear, steer_rad, mspd_rpm, direct);
      }

      float MonorailModel::LonAcceleration2ForceNeeded(float target_accel, float Fst, float Fr, float Flx, float steer_rad)
      {
        DynaParams params;
        {
          std::unique_lock<std::mutex> lck(mutex_);
          params = vehdyna_params_;
        }
        return static_cast<float>(target_accel * (params.veh_mass_kg_ + params.veh_inertia_kgm2_ * pow(steer_rad, 2) / pow(params.veh_wheel_base_, 2)) - (Fst - 0.5 * Fr * (2 + pow(steer_rad, 2)) - Flx));
      }

      void MonorailModel::ForceNeeded2Act(float &act_throttle, float &act_brake, float force_needed, bool stop_to_go, int gear, float steer_rad, float mspd_rpm, int direct)
      {
        float Z = 0;
        float B = 0;
        if (direct == components::Stopped)
        {
          if (false == stop_to_go) //brake
          {
            Z = 0;
            B = 2 * force_needed / (2 + pow(steer_rad, 2));
          }
          else //throttle
          {
            Z = force_needed;
            B = 0;
          }
        }
        else
        {
          float Zmax = Throttle2Z(100, gear, mspd_rpm, direct);
          float Zmin = Throttle2Z(-100, gear, mspd_rpm, direct);

          if (gear == components::Gear_D)
          {
            if (force_needed >= Zmax)
            {
              Z = Zmax;
              B = 2 * (force_needed - Zmax) / (2 + pow(steer_rad, 2));
            }
            else if (force_needed <= Zmin)
            {
              Z = Zmin;
              B = 2 * (force_needed - Zmin) / (2 + pow(steer_rad, 2));
            }
            else
            {
              Z = force_needed;
              B = 0;
            }
          }
          else if (gear == components::Gear_R)
          {
            if (force_needed <= Zmax)
            {
              Z = Zmax;
              B = 2 * (force_needed - Zmax) / (2 + pow(steer_rad, 2));
            }
            else if (force_needed >= Zmin)
            {
              Z = Zmin;
              B = 2 * (force_needed - Zmin) / (2 + pow(steer_rad, 2));
            }
            else
            {
              Z = force_needed;
              B = 0;
            }
          }
          else
          {
            Z = 0;
            B = 2 * force_needed / (2 + pow(steer_rad, 2));
          }
        }
        act_throttle = Z2Throttle(Z, gear, mspd_rpm, direct);
        act_brake = B2Press(B, direct);
      }

      float MonorailModel::LatDynamicInverse(float target_curv, float vspd_mps)
      {
        DynaParams params;
        {
          std::unique_lock<std::mutex> lck(mutex_);
          params = vehdyna_params_;
        }
        return atan(params.veh_wheel_base_ * target_curv);
      }

      void MonorailModel::MotionModelVspd(float &vspd_mps, float mspd_rpm)
      {
        float fl_wspd_mps = 0;
        float fr_wspd_mps = 0;
        float bl_wspd_mps = 0;
        float br_wspd_mps = 0;
        MotionModelWspd(fl_wspd_mps, fr_wspd_mps, bl_wspd_mps, br_wspd_mps, mspd_rpm, 0);
        vspd_mps = 0.5 * (bl_wspd_mps + br_wspd_mps);
      }

      void MonorailModel::MotionModelWspd(float &front_left_wspd_mps,
                                          float &front_right_wspd_mps,
                                          float &back_left_wspd_mps,
                                          float &back_right_wspd_mps,
                                          float mspd_rpm, float steer_rad)
      {
        DynaParams params;
        {
          std::unique_lock<std::mutex> lck(mutex_);
          params = vehdyna_params_;
        }
        float spd_gear = gearbox_->DirectModelSpd(0, mspd_rpm);
        float spd_reducer = reducer_->DirectModelSpd(spd_gear);
        float v_ref = 0.5 * (back_left_wheel_->DirectModelSpd(spd_reducer) + back_right_wheel_->DirectModelSpd(spd_reducer));
        float curv = LatCurvature(steer_rad);
        back_left_wspd_mps = (1 - 0.5 * params.veh_track_width_ * curv) * v_ref;
        back_right_wspd_mps = (1 + 0.5 * params.veh_track_width_ * curv) * v_ref;
        front_left_wspd_mps = sqrt(pow(1 - 0.5 * params.veh_track_width_ * curv, 2) + pow(params.veh_wheel_base_ * curv, 2)) * v_ref;
        front_right_wspd_mps = sqrt(pow(1 + 0.5 * params.veh_track_width_ * curv, 2) + pow(params.veh_wheel_base_ * curv, 2)) * v_ref;
      }

      float MonorailModel::Throttle2Z(float throttle, int gear, float mspd_rpm, int direct)
      {
        float motor_torq = motor_->DirectModel(throttle, gear, mspd_rpm, direct);
        float reducer_tor_out = reducer_->DirectModelTorq(gearbox_->DirectModelTorq(0, motor_torq));
        float back_left_wheel_torq = 0.5 * reducer_tor_out;
        float back_right_wheel_torq = 0.5 * reducer_tor_out;
        return back_left_wheel_->DirectModelTorq2Force(back_left_wheel_torq) + back_right_wheel_->DirectModelTorq2Force(back_right_wheel_torq);
      }

      float MonorailModel::Z2Throttle(float Z, int gear, float mspd_rpm, int direct)
      {
        float back_left_wheel_torq = back_left_wheel_->InverseModelForce2Torq(Z / 2);
        float back_right_wheel_torq = back_right_wheel_->InverseModelForce2Torq(Z / 2);
        float gear_toq_out = reducer_->InverseModelTorq(back_left_wheel_torq + back_right_wheel_torq);
        float motor_toq = gearbox_->InverseModelTorq(0, gear_toq_out);
        return motor_->InverseModel(motor_toq, gear, mspd_rpm, direct);
      }

      float MonorailModel::Pitch2Fst(float pitch_rad)
      {
        DynaParams params;
        {
          std::unique_lock<std::mutex> lck(mutex_);
          params = vehdyna_params_;
        }
        return params.veh_mass_kg_ * G * sin(pitch_rad);
      }

      float MonorailModel::Vspd2Flx(float vspd_mps, int direct)
      {
        DynaParams params;
        {
          std::unique_lock<std::mutex> lck(mutex_);
          params = vehdyna_params_;
        }
        int k_direct = direct == components::Forward    ? 1
                       : direct == components::Backward ? -1
                                                        : 0;
        return -1 * k_direct * params.air_res_eq_coe_ * pow(vspd_mps, 2);
      }

      float MonorailModel::Direct2Fr(int direct)
      {
        DynaParams params;
        {
          std::unique_lock<std::mutex> lck(mutex_);
          params = vehdyna_params_;
        }
        int k_direct = direct == components::Forward    ? 1
                       : direct == components::Backward ? -1
                                                        : 0;
        return -1 * k_direct * params.roll_res_coe_ * params.veh_mass_kg_ * G;
      }

      float MonorailModel::Press2BAvailable(float press)
      {
        float back_left_brake_torq_available_abs = back_left_brake_->DirectModelPress2Torq(press);
        float back_right_brake_torq_available_abs = back_right_brake_->DirectModelPress2Torq(press);
        float front_left_brake_torq_available_abs = front_left_brake_->DirectModelPress2Torq(press);
        float front_right_brake_torq_available_abs = front_left_brake_->DirectModelPress2Torq(press);
        return (front_left_wheel_->DirectModelTorq2Force(front_left_brake_torq_available_abs) + front_right_wheel_->DirectModelTorq2Force(front_right_brake_torq_available_abs) + back_left_wheel_->DirectModelTorq2Force(back_left_brake_torq_available_abs) + back_right_wheel_->DirectModelTorq2Force(back_right_brake_torq_available_abs));
      }

      float MonorailModel::B2Press(float B, int direct)
      {
        int k_direct = direct == components::Forward    ? 1
                       : direct == components::Backward ? -1
                                                        : 0;
        if (B * k_direct > 0)
          return 0;

        float bl_wheel_brk_torq_needed = back_left_wheel_->InverseModelForce2Torq(B / 4);
        float br_wheel_brk_torq_needed = back_right_wheel_->InverseModelForce2Torq(B / 4);
        float fl_wheel_brk_torq_needed = front_left_wheel_->InverseModelForce2Torq(B / 4);
        float fr_wheel_brk_torq_needed = front_right_wheel_->InverseModelForce2Torq(B / 4);
        float bl_press = back_left_brake_->InverseModelTorq2Press(fabs(bl_wheel_brk_torq_needed));
        float br_press = back_right_brake_->InverseModelTorq2Press(fabs(br_wheel_brk_torq_needed));
        float fl_press = front_left_brake_->InverseModelTorq2Press(fabs(fl_wheel_brk_torq_needed));
        float fr_press = front_right_brake_->InverseModelTorq2Press(fabs(fr_wheel_brk_torq_needed));
        float brake_needed = 0;
        brake_needed = std::max<float>(brake_needed, bl_press);
        brake_needed = std::max<float>(brake_needed, br_press);
        brake_needed = std::max<float>(brake_needed, fl_press);
        brake_needed = std::max<float>(brake_needed, fr_press);
        return brake_needed;
      }

      int MonorailModel::Direct(void *const stat)
      {
        ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
        return ecar_stat->veh_dyna_stat_msg.cdcu_veh_rundir.value == 1   ? 1
               : ecar_stat->veh_dyna_stat_msg.cdcu_veh_rundir.value == 2 ? -1
                                                                         : 0;
      }

      float MonorailModel::LonVspdMps(void *const stat)
      {
        ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
        return (ecar_stat->veh_dyna_stat_msg.cdcu_veh_longtdnalspd.value / 3.6);
      }

      float MonorailModel::LonAccelMps2(void *const stat)
      {
        ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
        return ecar_stat->imu_stat_msg.ins570d_acc_stat_msg.imu_accelx.value * 9.8;
      }

      float MonorailModel::LatAccelMps2(void *const stat)
      {
        ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
        return ecar_stat->imu_stat_msg.ins570d_acc_stat_msg.imu_accely.value * 9.8;
      }

      float MonorailModel::Yaw(void *const stat)
      {
        ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
        return ecar_stat->imu_stat_msg.ins570d_attitude_stat_msg.imu_rtkatt_yaw.value * M_PI / 180.0;
      }

      float MonorailModel::Pitch(void *const stat)
      {
        ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
        return ecar_stat->imu_stat_msg.ins570d_attitude_stat_msg.imu_rtkatt_pitch.value * M_PI / 180.0;
      }

      float MonorailModel::Roll(void *const stat)
      {
        ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
        return ecar_stat->imu_stat_msg.ins570d_attitude_stat_msg.imu_rtkatt_roll.value * M_PI / 180.0;
      }

      //Private Functions
      float MonorailModel::LonAcceleration(float Z, float B, float Fst, float Fr, float Flx, float steer_rad)
      {
        DynaParams params;
        {
          std::unique_lock<std::mutex> lck(mutex_);
          params = vehdyna_params_;
        }
        return static_cast<float>(Z + 0.5 * B * (2 + pow(steer_rad, 2)) + Fst - 0.5 * Fr * (2 + pow(steer_rad, 2)) - Flx) / (params.veh_mass_kg_ + params.veh_inertia_kgm2_ * pow(steer_rad, 2) / pow(params.veh_wheel_base_, 2));
      }

      float MonorailModel::LatCurvature(float steer_rad)
      {
        DynaParams params;
        {
          std::unique_lock<std::mutex> lck(mutex_);
          params = vehdyna_params_;
        }
        return tan(steer_rad) / params.veh_wheel_base_;
      }

    }
  }
} // namespace nirvana
