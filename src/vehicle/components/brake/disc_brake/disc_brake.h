#pragma once

#include <string>
#include <mutex>
#include <atomic>

#include "../Ibrake.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace brake
      {
        class DiscBrake : public IBrake
        {
        public:
          DiscBrake() {}
          virtual ~DiscBrake() {}
          virtual bool Init(float K_press2torq, float max_press) override;
          virtual float GetKpress2torq() const override;
          virtual float DirectModelPress2Torq(float brk_press) override;
          virtual float InverseModelTorq2Press(float target_torq) override;

          virtual float RealBrkPressBar(void *const stat) override;
          virtual uint8_t WorkMode(void *const stat) override;
          virtual uint8_t FaultLevel(void *const stat) override;
          virtual void AutoBrake(double stamp, enum BrakeMode brake_mode, float target_val, void *const cmd) override;
          virtual void CldBrake(double stamp, enum BrakeMode brake_mode, float target_val, void *const cmd) override;

        private:
          std::atomic<float> K_press2torq_;
          std::atomic<float> max_press_;
        };
      } // namespace motor
    }   // namespace dynamic
  }     // namespace control
} // namespace nirvana