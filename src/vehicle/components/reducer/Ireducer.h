#pragma once

#include <string>
#include <map>

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace reducer
      {
        class IReducer
        {
        public:
          virtual bool Init(float efficiency, float ratio) = 0;
          virtual float GetRatio() const = 0;
          virtual float GetEfficiency() const = 0;
          virtual float DirectModelTorq(float torq_in) = 0;
          virtual float DirectModelSpd(float spd_rpm_in) = 0;
          virtual float InverseModelTorq(float target_torq) = 0;
          virtual float InverseModelSpd(float target_spd_rpm) = 0;
        };
      } // namespace motor
    }   // namespace dynamic
  }     // namespace control
} // namespace nirvana