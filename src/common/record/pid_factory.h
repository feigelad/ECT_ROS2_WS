#pragma once

#include <memory>

// #include "ILog.h"
#include "Ipid.h"
namespace nirvana
{
  namespace common
  {
    namespace algorithm
    {
      class PidFactory
      {
      public:
        IPid *CreatePosPid();
      };
    } // namespace log
  }   // namespace common
} // namespace nirvana