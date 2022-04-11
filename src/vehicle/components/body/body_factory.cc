#include "body_factory.h"
#include "standard_body/standard_body.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace body
      {
        IBody *BodyFactory::CreateStandardBody()
        {
          return new body::StandardBody();
        }
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana