#include "rtk_factory.h"
#include "m2/rtk_m2.h"
#include "ins570d/rtk_ins570d.h"
namespace nirvana
{
  namespace location
  {
    Rtk *RtkFactory::CreateRtkM2()
    {
      return new Rtk_M2();
    }

    Rtk *RtkFactory::CreateRtkIns570d()
    {
      return new Rtk_Ins570d();
    }
  } // namespace message
} // namespace nirvana