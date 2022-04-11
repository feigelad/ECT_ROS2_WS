#pragma once

#include "Imotionstat.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace status
    {
      class MotionFactory
      {
      public:
        IMotionStat *CreateEcarMotion();
      };
    } // namespace dynamic
  }   // namespace control
} // namespace nirvana