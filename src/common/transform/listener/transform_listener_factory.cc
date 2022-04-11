
#include "transform_listener_factory.h"

namespace nirvana
{
  namespace common
  {
    namespace transform
    {
      ITransformListener *TransformListenerFactory::CreateTransformListener()
      {
        return new RosTransformListener();
      }
    } // namespace communication
  }   // namespace common
} // namespace nirvana