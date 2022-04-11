#include "wheel_factory.h"
#include "wheel1/wheel1.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace wheel
      {
        IWheel *WheelFactory::CreateWheel1()
        {
          return new wheel::Wheel1();
        }
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana