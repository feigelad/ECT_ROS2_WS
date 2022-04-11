#pragma once

#include <vector>
#include <map>

#include "../../dynamic/Idynamic.h"
#include "../../../common/algorithm/pid/Ipid.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace control
    {
      namespace controller
      {
        struct LonCtrlParams
        {
          LonCtrlParams() {}
          float kp_;
          float ki_;
          float kd_;
        };

        struct LatCtrlParams
        {
          LatCtrlParams() {}
          std::vector<float> t_pre_vec_;
          float kp_;
          float ki_;
          float kd_;
        };

        class IController
        {
        public:
          virtual bool Init(const LonCtrlParams &lon_params, const LatCtrlParams &lat_params, const std::shared_ptr<dynamic::IDynamic> &dynamic, const std::shared_ptr<common::algorithm::IPid> &force_pid) = 0;
          virtual void LonAccelControl(float &act_throttle, float &act_brake, float target_accel, float real_accel, int gear, float steer_rad, float pitch_rad, float mspd_rpm, float vspd_mps, int direct, double stamp) = 0;
          virtual void LatCurvatureControl(float &act_steer_rad, float target_curv, float vspd_mps) = 0;
          virtual void LonVspdControl(float &act_throttle, float &act_brake, std::map<int, float> target_vspd_list, int gear, float steer_rad, float pitch_rad, float mspd_rpm, float vspd_mps, int direct) = 0;
          virtual void TrajectoryFollower(float &act_throttle, float act_gear, float &act_brake, float &act_steer_rad, void *const traj, int target_direct) = 0;
        };
      }
    }
  }
}