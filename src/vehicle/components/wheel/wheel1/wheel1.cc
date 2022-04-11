#include "wheel1.h"
#include <vector>
#include <math.h>

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace wheel
      {
        bool Wheel1::Init(float efficiency, float radius)
        {
          efficiency_.exchange(efficiency);
          radius_.exchange(radius);
          return true;
        }

        float Wheel1::GetRadius() const
        {
          return radius_.load();
        }
        float Wheel1::GetEfficiency() const
        {
          return efficiency_.load();
        }

        float Wheel1::DirectModelTorq2Force(float torq_in)
        {
          if (radius_.load() == 0)
            return 0;

          return torq_in * efficiency_.load() / radius_.load();
        }
        float Wheel1::DirectModelSpd(float spd_rpm_in)
        {
          return radius_.load() * spd_rpm_in * 2 * M_PI / 60;
        }
        float Wheel1::InverseModelForce2Torq(float target_force)
        {
          if (efficiency_.load() == 0)
            return 0;

          return target_force * radius_.load() / efficiency_.load();
        }
        float Wheel1::InverseModelSpd(float target_spd_mps)
        {
          if (radius_.load() == 0)
            return 0;

          return target_spd_mps * (60 / (2 * M_PI)) / radius_.load();
        }
      }
    }
  }
}