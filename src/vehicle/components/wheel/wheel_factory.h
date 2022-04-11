#pragma once

#include "Iwheel.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace wheel
      {
        class WheelFactory
        {
        public:
          IWheel *CreateWheel1();
        };
      }

    } // namespace canbus
  }   // namespace message
} // namespace nirvana