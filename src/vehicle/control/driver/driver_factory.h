#pragma once

#include "Idriver.h"
namespace nirvana
{
  namespace vehicle
  {
    namespace control
    {
      namespace driver
      {
        class DriverFactory
        {
        public:
          IDriver *CreateEcarDriver();
        };
      }
    } // namespace control
  }
} // namespace nirvana