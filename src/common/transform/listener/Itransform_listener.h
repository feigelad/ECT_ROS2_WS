#pragma once
#include <string>

#include "../../systime/ITime.h"
#include "../../syslog/ILog.h"

namespace nirvana
{
  namespace common
  {
    namespace transform
    {
      enum Data2TransType
      {
        PointTransform = 0,
        PointStampedTransform,
        Vector3Transform,
        Vector3StampedTransform,
        PoseTransform,
        QuaternionTransform,
        QuaternionStampedTransform
      };
      class ITransformListener
      {
      public:
        virtual bool Init(void* node, 
                          std::string frame_id,
                          std::string child_frame_id,
                          std::shared_ptr<common::time::ITime> timer,
                          std::shared_ptr<common::log::ILog> logger) = 0;
        virtual bool Transform(enum Data2TransType type, const void *const src, void *const dest) = 0;
      };
    }
  }
}