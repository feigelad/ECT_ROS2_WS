#pragma once

#include <string>
#include <mutex>
#include <atomic>

#include "../Ireducer.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace reducer
      {
        class Reducer1 : public IReducer
        {
        public:
          Reducer1()
              : ratio_(0), efficiency_(1.0) {}
          virtual ~Reducer1() {}
          virtual bool Init(float efficiency, float ratio) override;
          virtual float GetRatio() const override;
          virtual float GetEfficiency() const override;
          virtual float DirectModelTorq(float torq_in) override;
          virtual float DirectModelSpd(float spd_rpm_in) override;
          virtual float InverseModelTorq(float target_torq) override;
          virtual float InverseModelSpd(float target_spd_rpm) override;

        private:
          std::atomic<float> efficiency_;
          std::atomic<float> ratio_;
        };
      } // namespace motor
    }   // namespace dynamic
  }     // namespace control
} // namespace nirvana