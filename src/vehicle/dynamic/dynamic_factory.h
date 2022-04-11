#pragma once

#include "Idynamic.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace dynamic
    {
      class DynamicFactory
      {
      public:
        IDynamic *CreateMonorailModel();
      };
    } // namespace dynamic
  }   // namespace control
} // namespace nirvana