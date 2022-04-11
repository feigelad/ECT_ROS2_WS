#pragma once

// #include "ros/ros.h"
#include "../ITime.h"

namespace nirvana
{
  namespace common
  {
    namespace time
    {
      class RosSysTime : public ITime
      {
      public:
        virtual uint64_t Now2MSec();
        virtual double Now2Sec();
        virtual void FillTimeStamp(void *const stamp);
      };
    } // namespace time
  }   // namespace common
} // namespace nirvana