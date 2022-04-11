#pragma once

#include "../Iremoter.h"
#include <atomic>

namespace nirvana
{
  namespace vehicle
  {
    namespace control
    {
      namespace remoter
      {
        class WflyEt06 : public IRemoter
        {
        public:
          WflyEt06() { auto_mode_.exchange(false); }
          virtual ~WflyEt06() {}
          virtual bool Init(float freq_threshold, float chan1_default, float chan1_max, float chan1_min,
                            float chan2_default, float chan2_max, float chan2_min,
                            float chan3_default, float chan3_max, float chan3_min,
                            float chan4_default, float chan4_max, float chan4_min,
                            float chan5_default, float chan5_max, float chan5_min,
                            float chan6_default, float chan6_max, float chan6_min) override;
          virtual bool RemoterEnableRq(double stamp, void *const stat) override;
          virtual float TargetDrive(int gear, void *const stat) override;
          virtual int TargetGear(void *const stat) override;
          virtual float TargetSteer(void *const stat) override;
          virtual float TargetBrake(void *const stat) override;

        private:
          std::atomic<float> freq_threshold_;
          std::atomic<float> chan1_duty_default_;
          std::atomic<float> chan1_duty_range_max_;
          std::atomic<float> chan1_duty_range_min_;
          std::atomic<float> chan2_duty_default_;
          std::atomic<float> chan2_duty_range_max_;
          std::atomic<float> chan2_duty_range_min_;
          std::atomic<float> chan3_duty_default_;
          std::atomic<float> chan3_duty_range_max_;
          std::atomic<float> chan3_duty_range_min_;
          std::atomic<float> chan4_duty_default_;
          std::atomic<float> chan4_duty_range_max_;
          std::atomic<float> chan4_duty_range_min_;
          std::atomic<float> chan5_duty_default_;
          std::atomic<float> chan5_duty_range_max_;
          std::atomic<float> chan5_duty_range_min_;
          std::atomic<float> chan6_duty_default_;
          std::atomic<float> chan6_duty_range_max_;
          std::atomic<float> chan6_duty_range_min_;

          std::atomic<bool> auto_mode_;
        };
      }
    }
  }
}