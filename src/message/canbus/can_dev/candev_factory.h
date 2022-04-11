#pragma once

#include "Ican.h"

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      class CanDevFctory
      {
      public:
        ICan *CreateZlgCan();
      };
    } // namespace canbus
  }   // namespace message
} // namespace nirvana