#pragma once

#include <string>
#include <map>

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      const int Gear_N = 0;
      const int Gear_D = 1;
      const int Gear_R = -1;
      const int Stopped = 0;
      const int Forward = 1;
      const int Backward = -1;

      namespace motor
      {
        enum AccelMode
        {
          Acc_Throttle = 0,
          Acc_Accel,
          Acc_Speed
        };

        class IMotor
        {
        public:
          virtual bool Init(float k_m_throt, float k_m_brake, float mspd_fb_cutoff, const std::map<int, float> &p_m_map) = 0;
          virtual float GetThrotKm() const = 0;
          virtual float GetBrakeKm() const = 0;
          virtual float GetPm(float throt) = 0;
          virtual float DirectModel(float throttle, int gear, float mspd_rpm, int real_direct) = 0;
          virtual float InverseModel(float tgt_torq, int real_gear, float mspd_rpm, int real_direct) = 0;
          virtual void TorqRangeAvailable(float &torq_max, float &torq_min, float mspd_rpm, int real_direct) = 0;

          virtual uint8_t WorkMode(void *const stat) = 0;
          virtual uint8_t FaultLevel(void *const stat) = 0;
          virtual float RealThrottle(void *const stat) = 0;
          virtual float RealCurrentA(void *const stat) = 0;
          virtual float RealTorqueNm(void *const stat) = 0;
          virtual int RealGear(void *const stat) = 0;
          virtual int RealDirect(void *const stat) = 0;
          virtual float RealMspdRpm(void *const stat) = 0;
          virtual void AutoDrive(double stamp, enum AccelMode acc_mode, int gear, float drive_value, void *const cmd) = 0;
          virtual void CldDrive(double stamp, enum AccelMode acc_mode, int gear, float drive_value, void *const cmd) = 0;
        };
      } // namespace motor
    }   // namespace dynamic
  }     // namespace control
} // namespace nirvana