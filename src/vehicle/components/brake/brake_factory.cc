#include "brake_factory.h"
#include "disc_brake/disc_brake.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace brake
      {
        IBrake *BrakeFactory::CreateDiscBrake()
        {
          return new brake::DiscBrake();
        }
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana