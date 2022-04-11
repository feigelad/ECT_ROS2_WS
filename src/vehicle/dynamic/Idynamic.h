#pragma once

#include <string>
#include <memory>
#include "../components/motor/motor_factory.h"
#include "../components/gearbox/gearbox_factory.h"
#include "../components/reducer/reducer_factory.h"
#include "../components/wheel/wheel_factory.h"
#include "../components/brake/brake_factory.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace dynamic
    {
      struct DynaParams
      {
        DynaParams() {}
        DynaParams(float mass, float inertia, float wheel_base, float air_coe, float roll_coe)
            : veh_mass_kg_(mass), veh_inertia_kgm2_(inertia), veh_wheel_base_(wheel_base), air_res_eq_coe_(air_coe), roll_res_coe_(roll_coe) {}
        float veh_mass_kg_;
        float veh_inertia_kgm2_;
        float veh_wheel_base_;
        float veh_track_width_;
        float roll_res_coe_;
        float air_res_eq_coe_;
      };
      class IDynamic
      {
      public:
        virtual bool Init(const std::shared_ptr<components::motor::IMotor> &motor,
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
                          const DynaParams &params) = 0;
        virtual float LonDynamic(float throttle, int gear, float brake, float steer_rad, float pitch_rad, float mspd_rpm, float vspd_mps, int direct) = 0;
        virtual float LatDynamic(float steer_rad, float vspd_mps) = 0;
        virtual void LonDynamicInverse(float &throttle, float &brake, float target_accel, int gear, float steer_rad, float pitch_rad, float mspd_rpm, float vspd_mps, int direct) = 0;
        virtual float LonAcceleration2ForceNeeded(float target_accel, float Fst, float Fr, float Flx, float steer_rad) = 0;
        virtual void ForceNeeded2Act(float &act_throttle, float &act_brake, float force_needed, bool stop_to_go, int gear, float steer_rad, float mspd_rpm, int direct) = 0;
        virtual float LatDynamicInverse(float target_curv, float vspd_mps) = 0;
        virtual void MotionModelVspd(float &vspd_mps, float mspd_rpm) = 0;
        virtual void MotionModelWspd(float &front_left_wspd_mps,
                                     float &front_right_wspd_mps,
                                     float &back_left_wspd_mps,
                                     float &back_right_wspd_mps,
                                     float mspd_rpm, float steer_rad) = 0;
        virtual float Throttle2Z(float throttle, int gear, float mspd_rpm, int direct) = 0;
        virtual float Z2Throttle(float Z, int gear, float mspd_rpm, int direct) = 0;
        virtual float Pitch2Fst(float pitch_rad) = 0;
        virtual float Vspd2Flx(float vspd_mps, int direct) = 0;
        virtual float Direct2Fr(int direct) = 0;
        virtual float Press2BAvailable(float press) = 0;
        virtual float B2Press(float B, int direct) = 0;

        virtual int Direct(void *const stat) = 0;
        virtual float LonVspdMps(void *const stat) = 0;
        virtual float LonAccelMps2(void *const stat) = 0;
        virtual float LatAccelMps2(void *const stat) = 0;
        virtual float Yaw(void *const stat) = 0;
        virtual float Pitch(void *const stat) = 0;
        virtual float Roll(void *const stat) = 0;

        // virtual void ParamsEstimate() = 0;
      };
    } // namespace dynamic
  }   // namespace control
} // namespace nirvana