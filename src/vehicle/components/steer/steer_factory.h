#pragma once

#include "Isteer.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace steer
      {
        class SteerFactory
        {
        public:
          ISteer *CreateCEPS();
        };
      }

    } // namespace canbus
  }   // namespace message
} // namespace nirvana