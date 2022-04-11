#pragma once

#include <string>
#include <mutex>
#include <atomic>

#include "../Iwheel.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace wheel
      {
        class Wheel1 : public IWheel
        {
        public:
          Wheel1()
              : radius_(0), efficiency_(1.0) {}
          virtual ~Wheel1() {}
          virtual bool Init(float efficiency, float radius) override;
          virtual float GetRadius() const override;
          virtual float GetEfficiency() const override;
          virtual float DirectModelTorq2Force(float torq_in) override;
          virtual float DirectModelSpd(float spd_rpm_in) override;
          virtual float InverseModelForce2Torq(float target_force) override;
          virtual float InverseModelSpd(float target_spd_mps) override;

        private:
          std::atomic<float> efficiency_;
          std::atomic<float> radius_;
        };
      } // namespace motor
    }   // namespace dynamic
  }     // namespace control
} // namespace nirvana