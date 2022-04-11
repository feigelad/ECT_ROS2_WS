#include "gearbox1.h"
#include <vector>

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace gearbox
      {
        bool GearBox1::Init(float efficiency, const std::map<int, float> &gear_map)
        {
          efficiency_.exchange(efficiency);
          gear_map_ = gear_map;
          return true;
        }

        void GearBox1::SetGear(int gear)
        {
          gear_.exchange(gear);
        }
        float GearBox1::GetRatio(int gear)
        {
          std::map<int, float> gmap;
          {
            std::unique_lock<std::mutex> lck(mutex_);
            gmap = gear_map_;
          }
          auto itr = gmap.find(gear);
          if (itr == gmap.end())
            return 0;

          return gmap[gear];
        }
        float GearBox1::GetEfficiency() const
        {
          return efficiency_.load();
        }

        float GearBox1::DirectModelTorq(int gear, float torq_in)
        {
          float ratio = GetRatio(gear);
          if (ratio == 0)
            return 0;
          return torq_in * efficiency_.load() / ratio;
        }
        float GearBox1::DirectModelSpd(int gear, float spd_rpm_in)
        {
          return GetRatio(gear) * spd_rpm_in;
        }
        float GearBox1::InverseModelTorq(int gear, float target_torq)
        {
          if (efficiency_.load() == 0)
            return 0;

          return target_torq * GetRatio(gear) / efficiency_.load();
        }
        float GearBox1::InverseModelSpd(int gear, float target_spd_rpm)
        {
          float ratio = GetRatio(gear);
          if (ratio == 0)
            return 0;

          return target_spd_rpm / ratio;
        }
      }
    }
  }
}