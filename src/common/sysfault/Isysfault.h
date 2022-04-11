#pragma once

#include <string>
#include "common/syslog/ILog.h"

namespace nirvana
{
  namespace common
  {
    namespace sysfault
    {
      enum FaultLevel
      {
        FaultNone = 0,
        FaultLevelI,
        FaultLevelII,
        FaultLevelIII
      };

      class ISysFault
      {
        virtual bool Init(const std::shared_ptr<common::log::ILog> &logger) = 0;
        virtual void SetFault(enum FaultLevel lev, uint16_t fault, bool log_flag = false) = 0;
        virtual uint16_t GetFault() = 0;
        virtual FaultLevel GetFaultLevel() = 0;
        virtual std::string FaultDebugString() = 0;
      };
    } // namespace sysfault
  }   // namespace common
} // namespace nirvana