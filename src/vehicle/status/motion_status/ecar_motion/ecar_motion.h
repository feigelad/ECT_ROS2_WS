#pragma once

#include "../Imotionstat.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace status
    {
      class EcarMotion : public IMotionStat
      {
      public:
        EcarMotion() {}
        virtual ~EcarMotion() {}

        virtual bool Init() override;
        virtual float VehicleSpeedMps(void *const stat) override;
        virtual int VehicleRunDirect(void *const stat) override;

        virtual bool WheelSpeedFrontLeftValid(void *const stat) override;
        virtual float WheelSpeedFrontLeftMps(void *const stat) override;
        virtual bool WheelSpeedFrontRightValid(void *const stat) override;
        virtual float WheelSpeedFrontRightMps(void *const stat) override;
        virtual bool WheelSpeedBackLeftValid(void *const stat) override;
        virtual float WheelSpeedBackLeftMps(void *const stat) override;
        virtual bool WheelSpeedBackRightValid(void *const stat) override;
        virtual float WheelSpeedBackRightMps(void *const stat) override;

        virtual float LonAccelMps2(void *const stat) override;
        virtual float YawRateRadps(void *const stat) override;
      };
    }
  }
}