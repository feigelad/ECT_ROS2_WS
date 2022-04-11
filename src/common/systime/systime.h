#pragma once

#include <string>
#include "ITime.h"
#include "ros_systime/ros_systime.h"

#define ROSTIMER

namespace nirvana
{
  namespace common
  {
    namespace time
    {
      class SysTime
      {
      public:
        static ITime *GetInstance()
        {
#ifdef ROSTIMER
          static RosSysTime timer_inst_;
#else
          static RosSysTime timer_inst_;
#endif
          return static_cast<ITime *>(&timer_inst_);
        }

      private:
        SysTime();
      };
    } // namespace time
  }   // namespace common
} // namespace nirvana