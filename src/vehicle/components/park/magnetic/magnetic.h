#pragma once

#include <string>
#include <mutex>
#include <atomic>

#include "../Ipark.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace park
      {
        class Magnetic : public IPark
        {
        public:
          Magnetic() {}
          virtual ~Magnetic() {}
          virtual bool Init() override;

          virtual bool RealParkStat(void *const stat) override;
          virtual uint8_t WorkMode(void *const stat) override;
          virtual uint8_t FaultLevel(void *const stat) override;
          virtual void AutoPark(double stamp, bool target, void *const cmd) override;
          virtual void CldPark(double stamp, bool target, void *const cmd) override;

        private:
        };
      } // namespace motor
    }   // namespace dynamic
  }     // namespace control
} // namespace nirvana