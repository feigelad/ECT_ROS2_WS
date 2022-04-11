#pragma once

#include "rtk.h"

namespace nirvana
{
  namespace location
  {
    class RtkFactory
    {
    public:
      Rtk *CreateRtkM2();
      Rtk *CreateRtkIns570d();
    };
  } // namespace message
} // namespace nirvana