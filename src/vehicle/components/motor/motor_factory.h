#pragma once

#include "Imotor.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace motor
      {
        class MotorFactory
        {
        public:
          IMotor *CreateDBStar_2p2kw();
        };
      }

    } // namespace canbus
  }   // namespace message
} // namespace nirvana