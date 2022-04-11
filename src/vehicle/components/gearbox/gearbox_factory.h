#pragma once

#include "Igearbox.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace gearbox
      {
        class GearBoxFactory
        {
        public:
          IGearBox *CreateGearBox1();
        };
      }

    } // namespace canbus
  }   // namespace message
} // namespace nirvana