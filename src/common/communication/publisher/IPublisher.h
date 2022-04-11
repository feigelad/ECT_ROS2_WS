#pragma once

#include <string>

namespace nirvana
{
  namespace common
  {
    namespace communication
    {
      class IPublisher
      {
      public:
        virtual bool Init(void* node, std::string topic, int buff_size) = 0;
        virtual void Publish(void *const data) = 0;
        virtual std::string GetErrorString() = 0;
      };
    } // namespace communication
  }   // namespace common
} // namespace nirvana