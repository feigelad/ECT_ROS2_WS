#pragma once

#include <string>
#include <map>

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace brake
      {
        enum BrakeMode
        {
          Brk_Pedpos = 0,
          Brk_Press,
          Brk_Deccel,
          Brk_Speed
        };

        class IBrake
        {
        public:
          virtual bool Init(float K_press2torq, float max_press) = 0;
          virtual float GetKpress2torq() const = 0;
          virtual float DirectModelPress2Torq(float brk_press) = 0;
          virtual float InverseModelTorq2Press(float target_torq) = 0;

          virtual float RealBrkPressBar(void *const stat) = 0;
          virtual uint8_t WorkMode(void *const stat) = 0;
          virtual uint8_t FaultLevel(void *const stat) = 0;
          virtual void AutoBrake(double stamp, enum BrakeMode brake_mode, float target_val, void *const cmd) = 0;
          virtual void CldBrake(double stamp, enum BrakeMode brake_mode, float target_val, void *const cmd) = 0;
        };
      } // namespace motor
    }   // namespace dynamic
  }     // namespace control
} // namespace nirvana