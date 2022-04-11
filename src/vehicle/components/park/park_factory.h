#pragma once

#include "Ipark.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace park
      {
        class ParkFactory
        {
        public:
          IPark *CreateMagnetic();
        };
      }

    } // namespace canbus
  }   // namespace message
} // namespace nirvana