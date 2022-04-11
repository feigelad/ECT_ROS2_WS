#pragma once

#include "Ireducer.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace reducer
      {
        class ReducerFactory
        {
        public:
          IReducer *CreateReducer1();
        };
      }

    } // namespace canbus
  }   // namespace message
} // namespace nirvana