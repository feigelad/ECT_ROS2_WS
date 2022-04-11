
#include "controller_factory.h"
#include "dynamic_controller/dynamic_controller.h"
namespace nirvana
{
  namespace vehicle
  {
    namespace control
    {
      namespace controller
      {
        IController *ControllerFactory::CreateDynamicController()
        {
          return (new DynamicController());
        }
      }
    } // namespace log
  }   // namespace common
} // namespace nirvana