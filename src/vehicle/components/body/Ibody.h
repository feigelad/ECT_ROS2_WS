#pragma once

#include <string>
#include <map>

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace body
      {
        //Lights
        const uint8_t HEAD_LIGHT_ON = static_cast<uint8_t>(1 << 7);
        const uint8_t DOUBLE_FLASH_LIGHT_ON = static_cast<uint8_t>(1 << 6);
        const uint8_t TURN_LEFT_LIGHT_ON = static_cast<uint8_t>(1 << 5);
        const uint8_t TURN_RIGHT_LIGHT_ON = static_cast<uint8_t>(1 << 4);
        const uint8_t BACK_LIGHT_ON = static_cast<uint8_t>(1 << 3);
        const uint8_t BUZZER_ON = static_cast<uint8_t>(1 << 2);
        const uint8_t HORN_ON = static_cast<uint8_t>(1 << 1);
        const uint8_t RUN_LIGHT_ON = static_cast<uint8_t>(1);

        class IBody
        {
        public:
          virtual bool Init() = 0;

          virtual uint8_t WorkMode(void *const stat) = 0;
          virtual uint8_t FaultLevel(void *const stat) = 0;
          virtual bool HeadLampOn(void *const stat) = 0;
          virtual bool BackLampOn(void *const stat) = 0;
          virtual bool LeftSteerLampOn(void *const stat) = 0;
          virtual bool RightSteerLampOn(void *const stat) = 0;
          virtual bool HarzardOn(void *const stat) = 0;
          virtual bool BrakeLampOn(void *const stat) = 0;
          virtual bool KlaxonOn(void *const stat) = 0;

          virtual void AutoBodySwitch(double stamp, uint8_t body_ctrl, void *const cmd) = 0;
          virtual void CldBodySwitch(double stamp, uint8_t body_ctrl, void *const cmd) = 0;
        };
      } // namespace motor
    }   // namespace dynamic
  }     // namespace control
} // namespace nirvana