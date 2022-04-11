#pragma once

#include <string>
#include <mutex>
#include <atomic>

#include "../Imotor.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace motor
      {
        class MotorDBStar2p2kw : public IMotor
        {
        public:
          MotorDBStar2p2kw() {}
          virtual ~MotorDBStar2p2kw() {}
          virtual bool Init(float k_m_throt, float k_m_brake, float mspd_fb_cutoff, const std::map<int, float> &p_m_map) override;
          virtual float GetThrotKm() const override;
          virtual float GetBrakeKm() const override;
          virtual float GetPm(float throt) override;
          virtual float DirectModel(float throttle, int gear, float mspd_rpm, int real_direct) override;
          virtual float InverseModel(float tgt_torq, int real_gear, float mspd_rpm, int real_direct) override;
          virtual void TorqRangeAvailable(float &torq_max, float &torq_min, float mspd_rpm, int real_direct) override;

          virtual uint8_t WorkMode(void *const stat) override;
          virtual uint8_t FaultLevel(void *const stat) override;
          virtual float RealThrottle(void *const stat) override;
          virtual float RealCurrentA(void *const stat) override;
          virtual float RealTorqueNm(void *const stat) override { return 0; }
          virtual int RealGear(void *const stat) override;
          virtual int RealDirect(void *const stat) override;
          virtual float RealMspdRpm(void *const stat) override;
          virtual void AutoDrive(double stamp, enum AccelMode acc_mode, int gear, float drive_value, void *const cmd) override;
          virtual void CldDrive(double stamp, enum AccelMode acc_mode, int gear, float drive_value, void *const cmd) override;

        private:
          std::atomic<float> Km_throt_;
          std::atomic<float> Km_brake_;
          std::atomic<float> mspd_fd_cutoff_;
          mutable std::mutex mutex_;
          std::map<int, float> Pm_map_;
        };
      } // namespace motor
    }   // namespace dynamic
  }     // namespace control
} // namespace nirvana