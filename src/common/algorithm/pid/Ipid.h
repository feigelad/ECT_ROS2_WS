#pragma once

namespace nirvana
{
  namespace common
  {
    namespace algorithm
    {
      class IPid
      {
      public:
        virtual bool Init(float kp, float ki, float kd, float integ_sep_threshold, float out_limit_max, float out_limit_min) = 0;
        virtual float Control(float target, float real, double stamp) = 0;
      };
    }
  }
}