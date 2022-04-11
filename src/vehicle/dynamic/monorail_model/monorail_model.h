#pragma once

#include <string>
#include <mutex>
#include <atomic>
#include <memory>

#include "../Idynamic.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace dynamic
    {
      class MonorailModel : public IDynamic
      {
      public:
        MonorailModel() {}
        virtual ~MonorailModel() {}
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
                          const DynaParams &params) override;
        virtual float LonDynamic(float throttle,
                                 int gear,
                                 float brake,
                                 float steer_rad,
                                 float pitch_rad,
                                 float mspd_rpm,
                                 float vspd_mps,
                                 int direct) override;
        virtual float LatDynamic(float steer_rad, float vspd_mps) override;
        virtual void LonDynamicInverse(float &throttle,
                                       float &brake,
                                       float target_accel,
                                       int gear,
                                       float steer_rad,
                                       float pitch_rad,
                                       float mspd_rpm,
                                       float vspd_mps,
                                       int direct) override;
        virtual float LonAcceleration2ForceNeeded(float target_accel, float Fst, float Fr, float Flx, float steer_rad) override;
        virtual void ForceNeeded2Act(float &act_throttle, float &act_brake, float force_needed, bool stop_to_go, int gear, float steer_rad, float mspd_rpm, int direct) override;
        virtual float LatDynamicInverse(float target_curv, float vspd_mps) override;
        virtual void MotionModelVspd(float &vspd_mps, float mspd_rpm) override;
        virtual void MotionModelWspd(float &front_left_wspd_mps,
                                     float &front_right_wspd_mps,
                                     float &back_left_wspd_mps,
                                     float &back_right_wspd_mps,
                                     float mspd_rpm, float steer_rad) override;
        virtual float Throttle2Z(float throttle, int gear, float mspd_rpm, int direct) override;
        virtual float Z2Throttle(float Z, int gear, float mspd_rpm, int direct) override;
        virtual float Pitch2Fst(float pitch_rad) override;
        virtual float Vspd2Flx(float vspd_mps, int direct) override;
        virtual float Direct2Fr(int direct) override;
        virtual float Press2BAvailable(float press) override;
        virtual float B2Press(float B, int direct) override;

        virtual int Direct(void *const stat) override;
        virtual float LonVspdMps(void *const stat) override;
        virtual float LonAccelMps2(void *const stat) override;
        virtual float LatAccelMps2(void *const stat) override;
        virtual float Yaw(void *const stat) override;
        virtual float Pitch(void *const stat) override;
        virtual float Roll(void *const stat) override;

      public:
        const float G = 9.8;

      private:
        float LonAcceleration(float Z, float B, float Fst, float Fr, float Flx, float steer_rad);

        float LatCurvature(float steer_rad);

      private:
        std::shared_ptr<components::motor::IMotor>
            motor_;
        std::shared_ptr<components::gearbox::IGearBox> gearbox_;
        std::shared_ptr<components::reducer::IReducer> reducer_;
        std::shared_ptr<components::wheel::IWheel> front_left_wheel_;
        std::shared_ptr<components::wheel::IWheel> front_right_wheel_;
        std::shared_ptr<components::wheel::IWheel> back_left_wheel_;
        std::shared_ptr<components::wheel::IWheel> back_right_wheel_;
        std::shared_ptr<components::brake::IBrake> front_left_brake_;
        std::shared_ptr<components::brake::IBrake> front_right_brake_;
        std::shared_ptr<components::brake::IBrake> back_left_brake_;
        std::shared_ptr<components::brake::IBrake> back_right_brake_;
        mutable std::mutex mutex_;
        DynaParams vehdyna_params_;
      };
    } // namespace dynamic
  }   // namespace control
} // namespace nirvana