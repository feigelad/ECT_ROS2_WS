#include "vehicle_manager.h"
#include "vehicle/msg/vehicle_debug.hpp"

#include <math.h>

namespace nirvana
{
  namespace vehicle
  {
    bool VehicleManager::Init(const std::shared_ptr<common::log::ILog> &logger,
                              const std::shared_ptr<common::time::ITime> &timer,
                              const std::shared_ptr<components::motor::IMotor> &motor,
                              const std::shared_ptr<components::brake::IBrake> &brake,
                              const std::shared_ptr<components::steer::ISteer> &steer,
                              const std::shared_ptr<components::park::IPark> &park,
                              const std::shared_ptr<components::body::IBody> &body,
                              const std::shared_ptr<dynamic::IDynamic> &dynamic,
                              const std::shared_ptr<control::driver::IDriver> &driver,
                              const std::shared_ptr<control::controller::IController> &controller,
                              const std::shared_ptr<control::remoter::IRemoter> &remoter)
    {
      if (logger == nullptr)
        return false;
      logger_ = logger;

      if (timer == nullptr)
      {
        logger_->Log(nirvana::common::log::Log_Error, "VehicleManager init failed, timer is nullptr !");
        return false;
      }
      timer_ = timer;

      if (motor == nullptr)
      {
        logger_->Log(nirvana::common::log::Log_Error, "VehicleManager init failed, motor is nullptr !");
        return false;
      }
      motor_ = motor;

      if (brake == nullptr)
      {
        logger_->Log(nirvana::common::log::Log_Error, "VehicleManager init failed, brake is nullptr !");
        return false;
      }
      brake_ = brake;

      if (steer == nullptr)
      {
        logger_->Log(nirvana::common::log::Log_Error, "VehicleManager init failed, steer is nullptr !");
        return false;
      }
      steer_ = steer;

      if (park == nullptr)
      {
        logger_->Log(nirvana::common::log::Log_Error, "VehicleManager init failed, park is nullptr !");
        return false;
      }
      park_ = park;

      if (body == nullptr)
      {
        logger_->Log(nirvana::common::log::Log_Error, "VehicleManager init failed, body is nullptr !");
        return false;
      }
      body_ = body;

      if (dynamic == nullptr)
      {
        logger_->Log(nirvana::common::log::Log_Error, "VehicleManager init failed, dynamic is nullptr !");
        return false;
      }
      dynamic_ = dynamic;

      if (driver == nullptr)
      {
        logger_->Log(nirvana::common::log::Log_Error, "VehicleManager init failed, driver is nullptr !");
        return false;
      }
      driver_ = driver;

      if (controller == nullptr)
      {
        logger_->Log(nirvana::common::log::Log_Error, "VehicleManager init failed, controller is nullptr !");
        return false;
      }
      controller_ = controller;

      if (remoter == nullptr)
      {
        logger_->Log(nirvana::common::log::Log_Error, "VehicleManager init failed, remoter is nullptr !");
        return false;
      }
      remoter_ = remoter;

      logger_->Log(nirvana::common::log::Log_Info, "VehicleManager init successful.");
      is_inited_.exchange(true);
      return true;
    }

    void VehicleManager::ControlTask(void *const traj, void *const stat, void *const cmd)
    {
      //Mode Switch
      control::driver::DriveMode mode = control::driver::DM_Standby;
      if (remoter_->RemoterEnableRq(timer_->Now2Sec(), stat))
        mode = control::driver::DM_Automatic;

      // logger_->Log(nirvana::common::log::Log_Info, "mode is " + std::to_string((int)mode));

      int gear = motor_->RealGear(stat);
      float mspd_rpm = motor_->RealMspdRpm(stat);
      float real_steer_rad = steer_->RealStrAngelRad(stat);
      float vspd_mps = dynamic_->LonVspdMps(stat);
      float real_accel = dynamic_->LonAccelMps2(stat);
      float pitch_rad = dynamic_->Pitch(stat);
      int direct = dynamic_->Direct(stat);

      int target_gear = remoter_->TargetGear(stat);
      float target_accel = remoter_->TargetDrive(target_gear, stat);
      float target_steer_deg = remoter_->TargetSteer(stat);

      float act_throt = remoter_->TargetDrive(target_gear, stat);
      float act_brake = remoter_->TargetBrake(stat);
      float act_steer_deg = remoter_->TargetSteer(stat);

      // controller_->LonAccelControl(act_throt, act_brake, target_accel, real_accel, gear, real_steer_rad, pitch_rad, mspd_rpm, vspd_mps, direct, timer_->Now2Sec());

      driver_->Drive(mode, components::motor::Acc_Throttle, target_gear, act_throt, components::brake::Brk_Press, act_brake, components::steer::Str_Angle, act_steer_deg, false, 0, stat, cmd);
    }

    void VehicleManager::SimTask(void *const stat, void *const cmd, void *const debug)
    {
      ::vehicle::msg::VehicleDebug *veh_debug = (::vehicle::msg::VehicleDebug *)debug;

      float act_throttle = motor_->RealThrottle(stat);
      int act_gear = motor_->RealGear(stat);
      float act_brake = brake_->RealBrkPressBar(stat);
      float act_steer_rad = steer_->RealStrAngelRad(stat);
      float pitch_rad = dynamic_->Pitch(stat);
      float mspd_rpm = motor_->RealMspdRpm(stat);
      float vspd_mps = dynamic_->LonVspdMps(stat);
      int direct = motor_->RealDirect(stat);
      veh_debug->sim_accel_x = dynamic_->LonDynamic(act_throttle, act_gear, act_brake, act_steer_rad, pitch_rad, mspd_rpm, vspd_mps, direct);
      veh_debug->sim_accel_x_err = veh_debug->sim_accel_x - dynamic_->LonAccelMps2(stat);

      veh_debug->sim_accel_y = dynamic_->LatDynamic(act_steer_rad, vspd_mps) * pow(vspd_mps, 2.0);
      veh_debug->sim_accel_y_err = veh_debug->sim_accel_y - dynamic_->LatAccelMps2(stat);
      veh_debug->time_stamp = timer_->Now2Sec();
      // timer_->FillTimeStamp(&(veh_debug->header.stamp));
    }
  }
}
