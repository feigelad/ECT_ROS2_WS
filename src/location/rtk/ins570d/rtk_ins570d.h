#pragma once

#include <mutex>
#include "../rtk.h"

namespace nirvana
{
  namespace location
  {
    class Rtk_Ins570d : public Rtk
    {
    public:
      Rtk_Ins570d() {}
      virtual ~Rtk_Ins570d() {}
      virtual void Parse(const void *const chass_stat, void *const pose) override;
    };
  } // namespace location
} // namespace nirvana