#pragma once

#include <string>
#include <mutex>
#include <atomic>

#include "../Isteer.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace steer
      {
        class CEPS : public ISteer
        {
        public:
          CEPS() {}
          virtual ~CEPS() {}
          virtual bool Init() override;

          virtual float RealStrAngelRad(void *const stat) override;
          virtual uint8_t WorkMode(void *const stat) override;
          virtual uint8_t FaultLevel(void *const stat) override;
          virtual void AutoSteer(double stamp, enum SteerMode steer_mode, float target_val, void *const cmd) override;
          virtual void CldSteer(double stamp, enum SteerMode steer_mode, float target_val, void *const cmd) override;

        private:
        };
      } // namespace motor
    }   // namespace dynamic
  }     // namespace control
} // namespace nirvana