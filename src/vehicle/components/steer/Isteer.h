#pragma once

#include <string>
#include <map>

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace steer
      {
        enum SteerMode
        {
          Str_Angle = 0,
          Str_Curvature
        };

        class ISteer
        {
        public:
          virtual bool Init() = 0;

          virtual float RealStrAngelRad(void *const stat) = 0;
          virtual uint8_t WorkMode(void *const stat) = 0;
          virtual uint8_t FaultLevel(void *const stat) = 0;
          virtual void AutoSteer(double stamp, enum SteerMode steer_mode, float target_val, void *const cmd) = 0;
          virtual void CldSteer(double stamp, enum SteerMode steer_mode, float target_val, void *const cmd) = 0;
        };
      } // namespace motor
    }   // namespace dynamic
  }     // namespace control
} // namespace nirvana