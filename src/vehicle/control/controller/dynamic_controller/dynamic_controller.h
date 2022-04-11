#pragma once

#include <mutex>
#include <atomic>
#include "../Icontroller.h"
#include "../../../dynamic/dynamic_factory.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace control
    {
      namespace controller
      {
        class DynamicController : public IController
        {
        public:
          DynamicController()
          {
            is_inited_.exchange(false);
          }
          virtual ~DynamicController() {}

          virtual bool Init(const LonCtrlParams &lon_params, const LatCtrlParams &lat_params, const std::shared_ptr<dynamic::IDynamic> &dynamic, const std::shared_ptr<common::algorithm::IPid> &force_pid) override;
          virtual void LonAccelControl(float &act_throttle, float &act_brake, float target_accel, float real_accel, int gear, float steer_rad, float pitch_rad, float mspd_rpm, float vspd_mps, int direct, double stamp) override;
          virtual void LatCurvatureControl(float &act_steer_rad, float target_curv, float vspd_mps) override;
          virtual void LonVspdControl(float &act_throttle, float &act_brake, std::map<int, float> target_vspd_list, int gear, float steer_rad, float pitch_rad, float mspd_rpm, float vspd_mps, int direct) override;
          virtual void TrajectoryFollower(float &act_throttle, float act_gear, float &act_brake, float &act_steer_rad, void *const traj, int target_direct) override;

        private:
          std::shared_ptr<dynamic::IDynamic> dynamic_;
          std::shared_ptr<common::algorithm::IPid> force_pid_;
          std::atomic<bool> is_inited_;
          mutable std::mutex mutex_;
          LonCtrlParams lon_ctrl_params_;
          LatCtrlParams lat_ctrl_params_;
        };
      }
    }
  }
}