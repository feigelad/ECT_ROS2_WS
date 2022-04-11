#include "reducer1.h"
#include <vector>
#include <math.h>

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace reducer
      {
        bool Reducer1::Init(float efficiency, float ratio)
        {
          efficiency_.exchange(efficiency);
          ratio_.exchange(ratio);
          return true;
        }

        float Reducer1::GetRatio() const
        {
          return ratio_.load();
        }
        float Reducer1::GetEfficiency() const
        {
          return efficiency_.load();
        }

        float Reducer1::DirectModelTorq(float torq_in)
        {
          if (ratio_.load() == 0)
            return 0;

          return torq_in * efficiency_.load() / ratio_.load();
        }
        float Reducer1::DirectModelSpd(float spd_rpm_in)
        {
          return ratio_.load() * spd_rpm_in;
        }
        float Reducer1::InverseModelTorq(float target_torq)
        {
          if (efficiency_.load() == 0)
            return 0;

          return target_torq * ratio_.load() / efficiency_.load();
        }
        float Reducer1::InverseModelSpd(float target_spd_rpm)
        {
          if (ratio_.load() == 0)
            return 0;

          return target_spd_rpm / ratio_.load();
        }
      }
    }
  }
}