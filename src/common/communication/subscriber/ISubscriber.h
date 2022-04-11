#pragma once

#include <string>
#include <deque>
#include "../../syslog/ILog.h"

namespace nirvana
{
  namespace common
  {
    namespace communication
    {
      class ISubscriber
      {
      public:
        virtual bool Init(void* node, std::string topic, int buff_size, int buff_num) = 0;
        virtual void Start() = 0;
        virtual int GetDataQueue(int queue_size, void *const data) = 0;
        virtual void GetCurrentData(void *const data) = 0;
        virtual std::string GetErrorString() = 0;
      };
    } // namespace communication
  }   // namespace common
} // namespace nirvana
