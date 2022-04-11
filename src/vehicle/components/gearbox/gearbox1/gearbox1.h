#pragma once

#include <string>
#include <mutex>
#include <atomic>

#include "../Igearbox.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace gearbox
      {
        class GearBox1 : public IGearBox
        {
        public:
          GearBox1()
              : gear_(0), efficiency_(1.0) {}
          virtual ~GearBox1() {}
          virtual bool Init(float efficiency, const std::map<int, float> &gear_map) override;
          virtual void SetGear(int gear) override;
          virtual float GetRatio(int gear = 0) override;
          virtual float GetEfficiency() const override;
          virtual float DirectModelTorq(int gear, float torq_in) override;
          virtual float DirectModelSpd(int gear, float spd_rpm_in) override;
          virtual float InverseModelTorq(int gear, float target_torq) override;
          virtual float InverseModelSpd(int gear, float target_spd_rpm) override;

        private:
          std::atomic<float> efficiency_;
          std::atomic<int> gear_;
          mutable std::mutex mutex_;
          std::map<int, float> gear_map_;
        };
      } // namespace motor
    }   // namespace dynamic
  }     // namespace control
} // namespace nirvana