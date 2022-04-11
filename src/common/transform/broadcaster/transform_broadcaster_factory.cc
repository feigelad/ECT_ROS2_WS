
#include "transform_broadcaster_factory.h"

namespace nirvana
{
  namespace common
  {
    namespace transform
    {
      ITransformBroadcaster *TransformBroadcasterFactory::CreateTransformBroadcaster()
      {
        return new RosTransformBroadcaster();
      }
    } // namespace communication
  }   // namespace common
} // namespace nirvana