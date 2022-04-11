#include "gearbox_factory.h"
#include "gearbox1/gearbox1.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace gearbox
      {
        IGearBox *GearBoxFactory::CreateGearBox1()
        {
          return new gearbox::GearBox1();
        }
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana