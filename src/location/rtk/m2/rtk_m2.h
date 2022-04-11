#pragma once

#include <mutex>
#include "../rtk.h"


namespace nirvana
{
  namespace location
  {
    class Rtk_M2 : public Rtk
    {
    public:
      Rtk_M2() {}
      virtual ~Rtk_M2() {}
      virtual void Parse(const void *const chass_stat, void *const pose) override;
    };
  } // namespace location
} // namespace nirvana