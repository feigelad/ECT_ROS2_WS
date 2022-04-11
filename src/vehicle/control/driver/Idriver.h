#pragma once

#include <string>
#include <memory>
#include "../../../common/systime/ITime.h"
#include "../../../common/syslog/ILog.h"
#include "../../components/motor/Imotor.h"
#include "../../components/brake/Ibrake.h"
#include "../../components/steer/Isteer.h"
#include "../../components/park/Ipark.h"
#include "../../components/body/Ibody.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace control
    {
      namespace driver
      {
        enum DriveMode
        {
          DM_Standby = 0,
          DM_Remote,
          DM_Mannul,
          DM_Automatic,
          DM_Cloud,
          DM_Emergency
        };

        enum Gear
        {
          Gear_N = 0,
          Gear_D,
          Gear_R
        };

        class IDriver
        {
        public:
          virtual bool Init(std::shared_ptr<common::log::ILog> logger,
                            std::shared_ptr<common::time::ITime> timer,
                            std::shared_ptr<components::motor::IMotor> motor,
                            std::shared_ptr<components::brake::IBrake> brake,
                            std::shared_ptr<components::steer::ISteer> steer,
                            std::shared_ptr<components::park::IPark> park,
                            std::shared_ptr<components::body::IBody> body) = 0;
          virtual void Drive(enum DriveMode dm,
                             enum components::motor::AccelMode acc_mode, int gear, float acc_val,
                             enum components::brake::BrakeMode brk_mode, float brk_val,
                             enum components::steer::SteerMode str_mode, float str_val,
                             bool prk_enable, uint8_t body_ctrl,
                             const void *const stat, void *const cmd) = 0;
        };
      }

    } // namespace control
  }

} // namespace nirvana