#pragma once

#include <memory>

#include "Icontroller.h"
namespace nirvana
{
  namespace vehicle
  {
    namespace control
    {
      namespace controller
      {
        class ControllerFactory
        {
        public:
          IController *CreateDynamicController();
        };
      }
    } // namespace log
  }   // namespace common
} // namespace nirvana