#pragma once
#include "../Ipid.h"
#include <atomic>
namespace nirvana
{
  namespace common
  {
    namespace algorithm
    {
      class PosPid : public IPid
      {
      public:
        PosPid() {}
        virtual ~PosPid() {}
        virtual bool Init(float kp, float ki, float kd, float integ_sep_threshold, float out_limit_max, float out_limit_min) override;
        virtual float Control(float target, float real, double stamp) override;

      private:
        float Kp_;
        float Ki_;
        float Kd_;
        float integ_sep_threshold_;
        float output_limit_max_;
        float output_limit_min_;
      };
    }
  }
}
