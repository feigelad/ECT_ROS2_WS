#pragma once

#include <string>
#include <map>

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace park
      {
        class IPark
        {
        public:
          virtual bool Init() = 0;

          virtual bool RealParkStat(void *const stat) = 0;
          virtual uint8_t WorkMode(void *const stat) = 0;
          virtual uint8_t FaultLevel(void *const stat) = 0;
          virtual void AutoPark(double stamp, bool target, void *const cmd) = 0;
          virtual void CldPark(double stamp, bool target, void *const cmd) = 0;
        };
      } // namespace motor
    }   // namespace dynamic
  }     // namespace control
} // namespace nirvana