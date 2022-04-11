#pragma once
#include <atomic>
#include "../dynamic/Idynamic.h"
#include "../control/driver/Idriver.h"
#include "../control/controller/Icontroller.h"
#include "../control/remoter/Iremoter.h"
#include "../components/motor/Imotor.h"
#include "../components/brake/Ibrake.h"
#include "../components/steer/Isteer.h"
#include "../components/park/Ipark.h"
#include "../components/body/Ibody.h"

namespace nirvana
{
  namespace vehicle
  {
    class VehicleManager
    {
    public:
      VehicleManager()
      {
        is_inited_.exchange(false);
      }
      virtual ~VehicleManager() {}

      bool Init(const std::shared_ptr<common::log::ILog> &logger,
                const std::shared_ptr<common::time::ITime> &timer,
                const std::shared_ptr<components::motor::IMotor> &motor,
                const std::shared_ptr<components::brake::IBrake> &brake,
                const std::shared_ptr<components::steer::ISteer> &steer,
                const std::shared_ptr<components::park::IPark> &park,
                const std::shared_ptr<components::body::IBody> &body,
                const std::shared_ptr<dynamic::IDynamic> &dynamic,
                const std::shared_ptr<control::driver::IDriver> &driver,
                const std::shared_ptr<control::controller::IController> &controller,
                const std::shared_ptr<control::remoter::IRemoter> &remoter);

      void ControlTask(void *const traj, void *const stat, void *const cmd);
      void SimTask(void *const stat, void *const cmd, void *const debug);

    private:
      std::atomic<bool> is_inited_;
      std::shared_ptr<common::log::ILog> logger_;
      std::shared_ptr<common::time::ITime> timer_;
      std::shared_ptr<components::motor::IMotor> motor_;
      std::shared_ptr<components::brake::IBrake> brake_;
      std::shared_ptr<components::steer::ISteer> steer_;
      std::shared_ptr<components::park::IPark> park_;
      std::shared_ptr<components::body::IBody> body_;
      std::shared_ptr<dynamic::IDynamic> dynamic_;
      std::shared_ptr<control::driver::IDriver> driver_;
      std::shared_ptr<control::controller::IController> controller_;
      std::shared_ptr<control::remoter::IRemoter> remoter_;
    };
  }
}