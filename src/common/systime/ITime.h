#pragma once

#include <string>

namespace nirvana
{
  namespace common
  {
    namespace time
    {
      class ITime
      {
      public:
        virtual uint64_t Now2MSec() = 0;
        virtual double Now2Sec() = 0;
        virtual void FillTimeStamp(void *const stamp) = 0;
      };
    } // namespace time
  }   // namespace common
} // namespace nirvana