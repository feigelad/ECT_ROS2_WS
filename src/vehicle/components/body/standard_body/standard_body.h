#pragma once

#include <string>
#include <mutex>
#include <atomic>

#include "../Ibody.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace body
      {
        class StandardBody : public IBody
        {
        public:
          StandardBody() {}
          virtual ~StandardBody() {}
          virtual bool Init() override;

          virtual uint8_t WorkMode(void *const stat) override;
          virtual uint8_t FaultLevel(void *const stat) override;
          virtual bool HeadLampOn(void *const stat) override;
          virtual bool BackLampOn(void *const stat) override;
          virtual bool LeftSteerLampOn(void *const stat) override;
          virtual bool RightSteerLampOn(void *const stat) override;
          virtual bool HarzardOn(void *const stat) override;
          virtual bool BrakeLampOn(void *const stat) override;
          virtual bool KlaxonOn(void *const stat) override;

          virtual void AutoBodySwitch(double stamp, uint8_t body_ctrl, void *const cmd) override;
          virtual void CldBodySwitch(double stamp, uint8_t body_ctrl, void *const cmd) override;

        private:
        };
      } // namespace motor
    }   // namespace dynamic
  }     // namespace control
} // namespace nirvana