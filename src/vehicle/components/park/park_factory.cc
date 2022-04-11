#include "park_factory.h"
#include "magnetic/magnetic.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace park
      {
        IPark *ParkFactory::CreateMagnetic()
        {
          return new park::Magnetic();
        }
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana