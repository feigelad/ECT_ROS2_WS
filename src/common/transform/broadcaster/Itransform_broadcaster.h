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
      enum TfBroadcastType
      {
        Static_Transform_Broadcast = 0,
        Dynamic_Transform_Broadcast
      };

      class ITransformBroadcaster
      {
      public:
        virtual bool Init(void *node,
                          std::string frame_id,
                          std::string child_frame_id,
                          enum TfBroadcastType type,
                          std::shared_ptr<common::time::ITime> timer,
                          std::shared_ptr<common::log::ILog> logger) = 0;
        virtual void Broadcast(double stamp, float pos_x, float pos_y, float pos_z, float att_r, float att_p, float att_y) = 0;
        virtual void Broadcast(double stamp, float pos_x, float pos_y, float pos_z, float quat_x, float quat_y, float quat_z, float quat_w) = 0;
      };
    }
  }
}