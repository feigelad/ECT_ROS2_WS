#pragma once

#include <string>
#include <map>

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace gearbox
      {
        class IGearBox
        {
        public:
          virtual bool Init(float efficiency, const std::map<int, float> &gear_map) = 0;
          virtual void SetGear(int gear) = 0;
          virtual float GetRatio(int gear = 0) = 0;
          virtual float GetEfficiency() const = 0;
          virtual float DirectModelTorq(int gear, float torq_in) = 0;
          virtual float DirectModelSpd(int gear, float spd_rpm_in) = 0;
          virtual float InverseModelTorq(int gear, float target_torq) = 0;
          virtual float InverseModelSpd(int gear, float target_spd_rpm) = 0;
        };
      } // namespace motor
    }   // namespace dynamic
  }     // namespace control
} // namespace nirvana