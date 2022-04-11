#pragma once

namespace nirvana
{
  namespace vehicle
  {
    namespace status
    {
      class IMotionStat
      {
      public:
        virtual bool Init() = 0;
        virtual float VehicleSpeedMps(void *const stat) = 0;
        virtual int VehicleRunDirect(void *const stat) = 0;

        virtual bool WheelSpeedFrontLeftValid(void *const stat) = 0;
        virtual float WheelSpeedFrontLeftMps(void *const stat) = 0;
        virtual bool WheelSpeedFrontRightValid(void *const stat) = 0;
        virtual float WheelSpeedFrontRightMps(void *const stat) = 0;
        virtual bool WheelSpeedBackLeftValid(void *const stat) = 0;
        virtual float WheelSpeedBackLeftMps(void *const stat) = 0;
        virtual bool WheelSpeedBackRightValid(void *const stat) = 0;
        virtual float WheelSpeedBackRightMps(void *const stat) = 0;

        virtual float LonAccelMps2(void *const stat) = 0;
        virtual float YawRateRadps(void *const stat) = 0;
      };
    }
  }
}