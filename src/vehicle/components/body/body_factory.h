#pragma once

#include "Ibody.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace body
      {
        class BodyFactory
        {
        public:
          IBody *CreateStandardBody();
        };
      }

    } // namespace canbus
  }   // namespace message
} // namespace nirvana