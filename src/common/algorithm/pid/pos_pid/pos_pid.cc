#include "pos_pid.h"
#include <algorithm>
#include <math.h>

namespace nirvana
{
  namespace common
  {
    namespace algorithm
    {
      bool PosPid::Init(float kp, float ki, float kd, float integ_sep_threshold, float out_limit_max, float out_limit_min)
      {
        Kp_ = kp;
        Ki_ = ki;
        Kd_ = kd;
        integ_sep_threshold_ = integ_sep_threshold;
        output_limit_max_ = out_limit_max;
        output_limit_min_ = out_limit_min;
        return true;
      }

      float PosPid::Control(float target, float real, double stamp)
      {
        static double s_stamp_last = 0;
        static float s_err_last = 0;
        static float s_integ = 0;
        static float s_out_last = 0;

        float err = target - real;

        //time inc at range[0.01, 0.5]
        float time_inc = std::max<double>(0.01, std::min<double>(0.5, stamp - s_stamp_last));

        //integ seperate by err
        float k_integ = 0;
        if (s_out_last >= output_limit_max_)
        {
          if (fabs(err) < integ_sep_threshold_)
          {
            k_integ = 1;
            if (err < 0)
              s_integ += err * time_inc;
          }
        }
        else if (s_out_last <= output_limit_min_)
        {
          if (fabs(err) < integ_sep_threshold_)
          {
            k_integ = 1;
            if (err > 0)
              s_integ += err * time_inc;
          }
        }
        else
        {
          if (fabs(err) < integ_sep_threshold_)
          {
            k_integ = 1;
            s_integ += err * time_inc;
          }
        }

        //out ar
        float out = Kp_ * err + k_integ * Ki_ * s_integ + Kd_ * (err - s_err_last) / time_inc;

        //record
        s_stamp_last = stamp;
        s_err_last = err;
        s_out_last = out;

        return std::max<float>(output_limit_min_, std::min<float>(output_limit_max_, out));
      }
    }
  }
}
