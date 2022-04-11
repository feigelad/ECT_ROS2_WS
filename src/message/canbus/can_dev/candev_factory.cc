#include "candev_factory.h"
#include "zlg_can/zlg_can.h"

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      ICan *CanDevFctory::CreateZlgCan()
      {
        return new zlg_can::ZlgCan();
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana