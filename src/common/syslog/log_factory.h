#pragma once

#include <memory>

// #include "ILog.h"
#include "../syslog/roslog/roslog.h"
namespace nirvana
{
  namespace common
  {
    namespace log
    {
      class LogFactory
      {
      public:
        ILog *CreateRosLogger();
      };
    } // namespace log
  }   // namespace common
} // namespace nirvana