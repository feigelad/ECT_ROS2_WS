#include "driver_factory.h"
#include "ecar_driver/ecar_driver.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace control
    {
      namespace driver
      {
        IDriver *DriverFactory::CreateEcarDriver()
        {
          return new EcarDriver();
        }
      }
    } // namespace control
  }
} // namespace nirvana