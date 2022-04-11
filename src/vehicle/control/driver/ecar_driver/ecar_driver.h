#pragma once
#include <mutex>
#include "../Idriver.h"

#include "message/msg/ecar_chassis.hpp"

namespace nirvana
{
  namespace vehicle
  {
    namespace control
    {
      namespace driver
      {
        class EcarDriver : public IDriver
        {
        public:
          EcarDriver() {}
          virtual ~EcarDriver() {}
          virtual bool Init(std::shared_ptr<common::log::ILog> logger,
                            std::shared_ptr<common::time::ITime> timer,
                            std::shared_ptr<components::motor::IMotor> motor,
                            std::shared_ptr<components::brake::IBrake> brake,
                            std::shared_ptr<components::steer::ISteer> steer,
                            std::shared_ptr<components::park::IPark> park,
                            std::shared_ptr<components::body::IBody> body) override;
          virtual void Drive(enum DriveMode dm,
                             enum components::motor::AccelMode acc_mode, int gear, float acc_val,
                             enum components::brake::BrakeMode brk_mode, float brk_val,
                             enum components::steer::SteerMode str_mode, float str_val,
                             bool prk_enable, uint8_t body_ctrl,
                             const void *const stat, void *const cmd) override;

        private:
          void SetMode(uint8_t &auto_active, uint8_t &cld_active, enum DriveMode dm, ::message::msg::EcarChassisStat *veh_stat);
          void AutoActive(uint8_t active, double stamp, ::message::msg::EcarChassisCmd *const cmd);
          void CldActive(uint8_t active, double stamp, ::message::msg::EcarChassisCmd *const cmd);
          void ThrotAutoDrive(enum components::motor::AccelMode acc_mode, int gear, float acc_val, double stamp, ::message::msg::EcarChassisCmd *const cmd);
          void ThrotCldDrive(enum components::motor::AccelMode acc_mode, int gear, float acc_val, double stamp, ::message::msg::EcarChassisCmd *const cmd);
          void BrakeAutoDrive(enum components::brake::BrakeMode brk_mode, float brk_val, double stamp, ::message::msg::EcarChassisCmd *const cmd);
          void BrakeCldDrive(enum components::brake::BrakeMode brk_mode, float brk_val, double stamp, ::message::msg::EcarChassisCmd *const cmd);
          void SteerAutoDrive(enum components::steer::SteerMode str_mode, float str_val, double stamp, ::message::msg::EcarChassisCmd *const cmd);
          void SteerCldDrive(enum components::steer::SteerMode str_mode, float str_val, double stamp, ::message::msg::EcarChassisCmd *const cmd);
          void ParkAutoDrive(bool prk_enable, double stamp, ::message::msg::EcarChassisCmd *const cmd);
          void ParkCldDrive(bool prk_enable, double stamp, ::message::msg::EcarChassisCmd *const cmd);
          void BodyAutoDrive(uint8_t body_ctrl, double stamp, ::message::msg::EcarChassisCmd *const cmd);
          void BodyCldDrive(uint8_t body_ctrl, double stamp, ::message::msg::EcarChassisCmd *const cmd);

        private:
          std::shared_ptr<common::log::ILog> logger_;
          std::shared_ptr<common::time::ITime> timer_;

          std::shared_ptr<components::motor::IMotor> motor_;
          std::shared_ptr<components::brake::IBrake> brake_;
          std::shared_ptr<components::steer::ISteer> steer_;
          std::shared_ptr<components::park::IPark> park_;
          std::shared_ptr<components::body::IBody> body_;
        };
      }

    } // namespace control
  }
} // namespace nirvana