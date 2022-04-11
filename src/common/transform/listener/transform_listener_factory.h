#pragma once

#include "Itransform_listener.h"
#include "ros_transform_listener/ros_transform_listener.h"

namespace nirvana
{
  namespace common
  {
    namespace transform
    {
      class TransformListenerFactory
      {
      public:
        ITransformListener *CreateTransformListener();
      };
    } // namespace communication
  }   // namespace common
} // namespace nirvana