#include "dynamic_controller.h"
#include <math.h>

// #include "Vehicle/"

namespace nirvana
{
  namespace vehicle
  {
    namespace control
    {
      namespace controller
      {
        bool DynamicController::Init(const LonCtrlParams &lon_params, const LatCtrlParams &lat_params, const std::shared_ptr<dynamic::IDynamic> &dynamic, const std::shared_ptr<common::algorithm::IPid> &force_pid)
        {
          lon_ctrl_params_ = lon_params;
          lat_ctrl_params_ = lat_params;
          if (dynamic == nullptr)
          {
            is_inited_.exchange(false);
            return false;
          }
          dynamic_ = dynamic;

          if (force_pid == nullptr)
          {
            is_inited_.exchange(false);
            return false;
          }
          force_pid_ = force_pid;

          is_inited_.exchange(true);
          return true;
        }

        void DynamicController::LonAccelControl(float &act_throttle, float &act_brake, float target_accel, float real_accel, int gear, float steer_rad, float pitch_rad, float mspd_rpm, float vspd_mps, int direct, double stamp)
        {
          float pre_ctrl_throttle = 0;
          float pre_ctrl_brake = 0;
          float fb_ctrl_throttle = 0;
          float fb_ctrl_brake = 0;

          //Pre control
          float Fst = dynamic_->Pitch2Fst(pitch_rad);
          float Fr = dynamic_->Direct2Fr(direct);
          float Flx = dynamic_->Vspd2Flx(vspd_mps, direct);
          float pre_force_needed = dynamic_->LonAcceleration2ForceNeeded(target_accel, Fst, Fr, Flx, steer_rad);

          //Feedback control
          float fb_force_needed = force_pid_->Control(target_accel, real_accel, stamp);

          float force_needed = pre_force_needed + fb_force_needed;

          bool stop_to_go = false;
          if (direct == components::Stopped)
          {
            if (fabs(target_accel) < 0.01) //brake
              stop_to_go = false;
            else //throttle
              stop_to_go = true;
          }
          dynamic_->ForceNeeded2Act(act_throttle, act_brake, force_needed, stop_to_go, gear, steer_rad, mspd_rpm, direct);
        }

        void DynamicController::LatCurvatureControl(float &act_steer_rad, float target_curv, float vspd_mps)
        {
          //Pre control
          float pre_ctrl_steer = dynamic_->LatDynamicInverse(target_curv, vspd_mps);

          //feedback control
          float fb_ctrl_steer = 0;

          act_steer_rad = pre_ctrl_steer + fb_ctrl_steer;
        }

        void DynamicController::LonVspdControl(float &act_throttle, float &act_brake, std::map<int, float> target_vspd_list, int gear, float steer_rad, float pitch_rad, float mspd_rpm, float vspd_mps, int direct)
        {
        }

        void DynamicController::TrajectoryFollower(float &act_throttle, float act_gear, float &act_brake, float &act_steer_rad, void *const traj, int target_direct)
        {
        }
      }
    }
  }
}