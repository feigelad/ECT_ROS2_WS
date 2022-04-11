#pragma once

#include "Ibrake.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace brake
      {
        class BrakeFactory
        {
        public:
          IBrake *CreateDiscBrake();
        };
      }

    } // namespace canbus
  }   // namespace message
} // namespace nirvana