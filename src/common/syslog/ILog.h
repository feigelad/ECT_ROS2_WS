#pragma once

#include <string>
#include <memory>
#include "../../common/systime/systime.h"

namespace nirvana
{
  namespace common
  {
    namespace log
    {
      enum LogLevel
      {
        Log_Debug = 0,
        Log_Info,
        Log_Warn,
        Log_Error,
        Log_Fatal
      };

      class ILog
      {
      public:
        virtual bool Init(void *node, std::shared_ptr<common::time::ITime> timer) = 0;
        virtual void Log(LogLevel lev, const std::string &str) = 0;
        virtual void Log(LogLevel lev, const std::stringstream &strstr) = 0;
        // virtual ILog &operator<<()
      };
    } // namespace log
  }   // namespace common
} // namespace nirvana
