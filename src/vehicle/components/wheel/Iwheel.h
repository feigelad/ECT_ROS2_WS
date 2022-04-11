#pragma once

#include <string>
#include <map>

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace wheel
      {
        class IWheel
        {
        public:
          virtual bool Init(float efficiency, float radius) = 0;
          virtual float GetRadius() const = 0;
          virtual float GetEfficiency() const = 0;
          virtual float DirectModelTorq2Force(float torq_in) = 0;
          virtual float DirectModelSpd(float spd_rpm_in) = 0;
          virtual float InverseModelForce2Torq(float target_force) = 0;
          virtual float InverseModelSpd(float target_spd_mps) = 0;
        };
      } // namespace motor
    }   // namespace dynamic
  }     // namespace control
} // namespace nirvana