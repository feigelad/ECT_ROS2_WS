#include "steer_factory.h"
#include "ceps/ceps.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace steer
      {
        ISteer *SteerFactory::CreateCEPS()
        {
          return new steer::CEPS();
        }
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana