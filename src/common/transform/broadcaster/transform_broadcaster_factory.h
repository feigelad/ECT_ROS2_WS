#pragma once

#include "Itransform_broadcaster.h"
#include "ros_transform_broadcaster/ros_transform_broadcaster.h"

namespace nirvana
{
  namespace common
  {
    namespace transform
    {
      class TransformBroadcasterFactory
      {
      public:
        ITransformBroadcaster *CreateTransformBroadcaster();
      };
    } // namespace communication
  }   // namespace common
} // namespace nirvana