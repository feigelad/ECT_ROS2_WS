#pragma once

namespace nirvana
{
  namespace vehicle
  {
    namespace control
    {
      namespace remoter
      {
        class IRemoter
        {
        public:
          virtual bool Init(float freq_threshold, float chan1_default, float chan1_max, float chan1_min,
                            float chan2_default, float chan2_max, float chan2_min,
                            float chan3_default, float chan3_max, float chan3_min,
                            float chan4_default, float chan4_max, float chan4_min,
                            float chan5_default, float chan5_max, float chan5_min,
                            float chan6_default, float chan6_max, float chan6_min) = 0;
          virtual bool RemoterEnableRq(double stamp, void *const stat) = 0;
          virtual float TargetDrive(int gear, void *const stat) = 0;
          virtual int TargetGear(void *const stat) = 0;
          virtual float TargetSteer(void *const stat) = 0;
          virtual float TargetBrake(void *const stat) = 0;
        };
      }
    }
  }
}