#pragma once

#include <thread>

// #include "ros/ros.h"
// #include "dynamic_reconfigure/server.h"
//1233
#include <rclcpp/rclcpp.hpp>
#include "../../common/syslog/log_factory.h"
#include "../../common/communication/ipc_manager/ipc_manager.h"
#include "../../common/schedule/thread_manager/thread_manager.h"
#include "message/msg/ecar_chassis.hpp"
#include "vehicle/msg/trajectory.hpp"
#include "vehicle/msg/vehicle_debug.hpp"

#include "../components/motor/motor_factory.h"
#include "../components/gearbox/gearbox_factory.h"
#include "../components/reducer/reducer_factory.h"
#include "../components/wheel/wheel_factory.h"
#include "../components/steer/steer_factory.h"
#include "../components/park/park_factory.h"
#include "../components/brake/brake_factory.h"
#include "../components/body/body_factory.h"
#include "../dynamic/dynamic_factory.h"
#include "../../common/algorithm/pid/pid_factory.h"
#include "../control/controller/controller_factory.h"
#include "../control/driver/driver_factory.h"
#include "../control/remoter/remoter_factory.h"
#include "../vehicle_manager/vehicle_manager.h"

#include <deque>

using namespace nirvana::vehicle;

class VehicleNode : public rclcpp::Node
{
public:
  VehicleNode()
    :Node("vehicle_node") {}
  virtual ~VehicleNode()
  {
  }
  void onInit()
  {
    //data init
    is_inited_.exchange(false);
    is_message_manager_online_.exchange(false);
    //Get Systime Instance
    timer_ = std::shared_ptr<nirvana::common::time::ITime>(nirvana::common::time::SysTime::GetInstance());

    //Create logger
    nirvana::common::log::LogFactory log_factory;
    logger_ = std::shared_ptr<nirvana::common::log::ILog>(log_factory.CreateRosLogger());
    logger_->Init(this, timer_);

    //Create motor instance
    //TODO: params need to be confirmed
    float k_m_throt = 0.4922;
    float k_m_brake = 0.138;
    float mspd_fb_cutoff = 400;
    std::map<int, float> p_m_map;

    components::motor::MotorFactory motor_factory;
    std::shared_ptr<components::motor::IMotor> motor = std::shared_ptr<components::motor::IMotor>(motor_factory.CreateDBStar_2p2kw());
    if (false == motor->Init(k_m_throt, k_m_brake, mspd_fb_cutoff, p_m_map))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component motor init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Component motor init successful.");

    //Create gearbox instance
    //TODO: params need to be confirmed
    float efficiency = 1.0;
    std::map<int, float> gear_map;
    gear_map[0] = 1.0;
    components::gearbox::GearBoxFactory gearbox_factory;
    std::shared_ptr<components::gearbox::IGearBox> gearbox = std::shared_ptr<components::gearbox::IGearBox>(gearbox_factory.CreateGearBox1());
    if (false == gearbox->Init(efficiency, gear_map))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component gearbox init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Component gearbox init successful.");

    //Create reducer instance
    //TODO: params need to be confirmed
    float red_efficiency = 1.0;
    float ratio = 1 / 12.0;
    components::reducer::ReducerFactory reducer_factory;
    std::shared_ptr<components::reducer::IReducer> reducer = std::shared_ptr<components::reducer::IReducer>(reducer_factory.CreateReducer1());
    if (false == reducer->Init(red_efficiency, ratio))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component reducer init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Component reducer init successful.");

    //Create wheel instance
    //TODO: params need to be confirmed
    float wheel_efficiency = 1.0;
    float radius = 0.23;
    components::wheel::WheelFactory wheel_factory;
    std::shared_ptr<components::wheel::IWheel> blwheel = std::shared_ptr<components::wheel::IWheel>(wheel_factory.CreateWheel1());
    if (false == blwheel->Init(wheel_efficiency, radius))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component blwheel init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Component blwheel init successful.");

    std::shared_ptr<components::wheel::IWheel> brwheel = std::shared_ptr<components::wheel::IWheel>(wheel_factory.CreateWheel1());
    if (false == brwheel->Init(efficiency, radius))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component brwheel init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Component brwheel init successful.");

    std::shared_ptr<components::wheel::IWheel> flwheel = std::shared_ptr<components::wheel::IWheel>(wheel_factory.CreateWheel1());
    if (false == flwheel->Init(efficiency, radius))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component flwheel init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Component flwheel init successful.");

    std::shared_ptr<components::wheel::IWheel> frwheel = std::shared_ptr<components::wheel::IWheel>(wheel_factory.CreateWheel1());
    if (false == frwheel->Init(efficiency, radius))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component frwheel init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Component frwheel init successful.");

    //Create brake instance
    //TODO: params need to be confirmed
    float K_press2torq = 1.0;
    float max_press = 80;
    components::brake::BrakeFactory brake_factory;
    std::shared_ptr<components::brake::IBrake> blbrake = std::shared_ptr<components::brake::IBrake>(brake_factory.CreateDiscBrake());
    if (false == blbrake->Init(K_press2torq, max_press))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component blbrake init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Component blbrake init successful.");

    std::shared_ptr<components::brake::IBrake> brbrake = std::shared_ptr<components::brake::IBrake>(brake_factory.CreateDiscBrake());
    if (false == brbrake->Init(K_press2torq, max_press))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component brbrake init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Component brbrake init successful.");

    std::shared_ptr<components::brake::IBrake> flbrake = std::shared_ptr<components::brake::IBrake>(brake_factory.CreateDiscBrake());
    if (false == flbrake->Init(K_press2torq, max_press))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component flbrake init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Component flbrake init successful.");

    std::shared_ptr<components::brake::IBrake> frbrake = std::shared_ptr<components::brake::IBrake>(brake_factory.CreateDiscBrake());
    if (false == frbrake->Init(K_press2torq, max_press))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component frbrake init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Component frbrake init successful.");

    //Create steer instance
    //TODO: params need to be confirmed
    components::steer::SteerFactory steer_factory;
    std::shared_ptr<components::steer::ISteer> steer = std::shared_ptr<components::steer::ISteer>(steer_factory.CreateCEPS());
    if (false == steer->Init())
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component steer init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Component steer init successful.");

    //Create park instance
    //TODO: params need to be confirmed
    components::park::ParkFactory park_factory;
    std::shared_ptr<components::park::IPark> park = std::shared_ptr<components::park::IPark>(park_factory.CreateMagnetic());
    if (false == park->Init())
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component park init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Component park init successful.");

    //Create body instance
    //TODO: params need to be confirmed
    components::body::BodyFactory body_factory;
    std::shared_ptr<components::body::IBody> body = std::shared_ptr<components::body::IBody>(body_factory.CreateStandardBody());
    if (false == body->Init())
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component body init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Component body init successful.");

    //Create dynamic instance
    //TODO: params need to be confirmed
    dynamic::DynaParams params;
    params.veh_wheel_base_ = 1.0;
    params.veh_track_width_ = 0.8;
    params.veh_mass_kg_ = 600;
    params.veh_inertia_kgm2_ = 0;
    params.roll_res_coe_ = 0;
    params.air_res_eq_coe_ = 0;
    dynamic::DynamicFactory dynamic_factory;
    std::shared_ptr<dynamic::IDynamic> dynamic = std::shared_ptr<dynamic::IDynamic>(dynamic_factory.CreateMonorailModel());
    if (false == dynamic->Init(motor, gearbox, reducer, flwheel, frwheel, blwheel, brwheel, flbrake, frbrake, blbrake, brbrake, params))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component dynamic init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Component dynamic init successful.");

    //Create driver instance
    //TODO: params need to be confirmed
    control::driver::DriverFactory driver_factory;
    std::shared_ptr<control::driver::IDriver> driver = std::shared_ptr<control::driver::IDriver>(driver_factory.CreateEcarDriver());
    if (false == driver->Init(logger_, timer_, motor, blbrake, steer, park, body))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component driver init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Component driver init successful.");

    //Create controller instance
    //TODO: params need to be confirmed
    control::controller::LonCtrlParams lon_params;
    control::controller::LatCtrlParams lat_params;

    //PID params
    float kp = 0;
    float ki = 0;
    float kd = 0;
    float integ_sep_threshold = 0;
    float out_limit_max = 0;
    float out_limit_min = 0;
    nirvana::common::algorithm::PidFactory pid_factory;
    std::shared_ptr<nirvana::common::algorithm::IPid> force_pid = std::shared_ptr<nirvana::common::algorithm::IPid>(pid_factory.CreatePosPid());
    if (false == force_pid->Init(kp, ki, kd, integ_sep_threshold, out_limit_max, out_limit_min))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component force_pid init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Component force_pid init successful.");

    control::controller::ControllerFactory controller_factory;
    std::shared_ptr<control::controller::IController> controller = std::shared_ptr<control::controller::IController>(controller_factory.CreateDynamicController());
    if (false == controller->Init(lon_params, lat_params, dynamic, force_pid))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component controller init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Component controller init successful.");

    //Create remoter instance
    //TODO: params need to be confirmed
    float freq_threshold = 250;
    float chan1_default = 45.54;
    float chan1_max = 58.11;
    float chan1_min = 32.93;
    float chan2_default = 45.54;
    float chan2_max = 58.11;
    float chan2_min = 32.93;
    float chan3_default = 45.54;
    float chan3_max = 58.11;
    float chan3_min = 32.93;
    float chan4_default = 45.54;
    float chan4_max = 58.11;
    float chan4_min = 32.93;
    float chan5_default = 45.54;
    float chan5_max = 58.11;
    float chan5_min = 32.93;
    float chan6_default = 45.54;
    float chan6_max = 58.11;
    float chan6_min = 32.93;
    control::remoter::RemoterFactory remoter_factory;
    std::shared_ptr<control::remoter::IRemoter> remoter = std::shared_ptr<control::remoter::IRemoter>(remoter_factory.CreateWflyEt06());
    if (false == remoter->Init(freq_threshold, chan1_default, chan1_max, chan1_min,
                               chan2_default, chan2_max, chan2_min,
                               chan3_default, chan3_max, chan3_min,
                               chan4_default, chan4_max, chan4_min,
                               chan5_default, chan5_max, chan5_min,
                               chan6_default, chan6_max, chan6_min))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component remoter init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Component remoter init successful.");

    //Create VehicleManager instance
    std::shared_ptr<VehicleManager> vehicle_manager = std::shared_ptr<VehicleManager>(new VehicleManager());
    if (false == vehicle_manager->Init(logger_, timer_, motor, blbrake, steer, park, body, dynamic, driver, controller, remoter))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Component vehicle_manager init failed !");
      return;
    }
    vehicle_manager_ = vehicle_manager;
    logger_->Log(nirvana::common::log::Log_Info, "Component vehicle_manager init successful.");

    //Create Ipc Manager Instance
    ipc_manager_ = std::shared_ptr<nirvana::common::communication::IpcManager>(new nirvana::common::communication::IpcManager());
    if (false == ipc_manager_->Init(this, logger_))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Ipc manager init failed !");
      return;
    }

    //Register Ipc Messages
    //TODO:...
    if (false == ipc_manager_->RegistMsg<::message::msg::EcarChassisCmd>("EcarChassisCmdMsg", nirvana::common::communication::MsgPub))
    {
      logger_->Log(nirvana::common::log::Log_Error, "EcarChassisCmdMsg regist failed !");
      return;
    }
    if (false == ipc_manager_->RegistMsg<::message::msg::EcarChassis>("EcarChassisMsg", nirvana::common::communication::MsgSub))
    {
      logger_->Log(nirvana::common::log::Log_Error, "EcarChassisMsg regist failed !");
      return;
    }
    if (false == ipc_manager_->RegistMsg<::vehicle::msg::Trajectory>("TrajectoryMsg", nirvana::common::communication::MsgSub))
    {
      logger_->Log(nirvana::common::log::Log_Error, "TrajectoryMsg regist failed !");
      return;
    }
    if (false == ipc_manager_->RegistMsg<::vehicle::msg::VehicleDebug>("VehicleDebugMsg", nirvana::common::communication::MsgPub))
    {
      logger_->Log(nirvana::common::log::Log_Error, "VehicleDebugMsg regist failed !");
      return;
    }

    ipc_manager_->Start();

    //Task Manager
    task_manager_ = std::shared_ptr<nirvana::common::schedule::ThreadManager>(new nirvana::common::schedule::ThreadManager());
    if (task_manager_->Init("vehicle_node", timer_, logger_, ipc_manager_) == false)
    {
      logger_->Log(nirvana::common::log::Log_Error, "Task manager init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Task manager init successfully.");

    //Regist task threads
    if (false == task_manager_->RegistTaskThread("pub_thread", std::bind(&VehicleNode::PubTsk, this), 0.02, 0.05, 0.1))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Task manager regist pub_thread failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Task manager regist pub_thread successfully.");

    if (false == task_manager_->RegistTaskThread("sub_thread", std::bind(&VehicleNode::SubTsk, this), 0.01, 0.02, 0.05))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Task manager regist sub_thread failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Task manager regist sub_thread successfully.");

    if (false == task_manager_->RegistTaskThread("run_thread", std::bind(&VehicleNode::RunTsk, this), 0.02, 0.05, 0.2))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Task manager regist run_thread failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Task manager regist run_thread successfully.");

    logger_->Log(nirvana::common::log::Log_Info, "VehicleNode inited successfully .");
    is_inited_.exchange(true);
  }

public:
  void PubTsk()
  {
    if (is_message_manager_online_.load() == false)
      return;
    std::unique_lock<std::mutex> lck(mutex_);
    if (false == ipc_manager_->PubMsg("EcarChassisCmdMsg", &ecar_chassis_msg_.cmd_msg))
    {
      logger_->Log(nirvana::common::log::Log_Error, "IpcManager Pub EcarChassisCmdMsg failed !");
    }

    if (false == ipc_manager_->PubMsg("VehicleDebugMsg", &vehicle_debug_))
    {
      logger_->Log(nirvana::common::log::Log_Error, "IpcManager Pub VehicleDebugMsg failed !");
    }
  }

  void SubTsk()
  {
    //Sub EcarChassisMsg
    std::deque<::message::msg::EcarChassis> ecar_chass_queue;
    if (0 > ipc_manager_->SubMsg("EcarChassisMsg", 1, &ecar_chass_queue))
    {
      logger_->Log(nirvana::common::log::Log_Error, "IpcManager Sub message failed !");
    }
    if (false == ecar_chass_queue.empty())
    {
      if (false == is_message_manager_online_.load())
      {
        std::unique_lock<std::mutex> lck(mutex_);
        ecar_chassis_msg_.cmd_msg = ecar_chass_queue.back().cmd_msg;
      }
      is_message_manager_online_.exchange(true);
      std::unique_lock<std::mutex> lck(mutex_);
      ecar_chassis_msg_.stat_msg = ecar_chass_queue.back().stat_msg;
    }

    //Sub TrajectoryMsg
    std::deque<::vehicle::msg::Trajectory> trajectory_queue;
    if (0 > ipc_manager_->SubMsg("TrajectoryMsg", 1, &trajectory_queue))
    {
      logger_->Log(nirvana::common::log::Log_Error, "IpcManager Sub message failed !");
    }

    if (false == trajectory_queue.empty())
    {
      std::unique_lock<std::mutex> lck(mutex_);
      trajectory_ = trajectory_queue.back();
    }
  }

  void RunTsk()
  {
    if (is_message_manager_online_.load() == false)
      return;
    ::message::msg::EcarChassisCmd ecar_cmd;
    ::message::msg::EcarChassisStat ecar_stat;
    ::vehicle::msg::Trajectory traj;
    ::vehicle::msg::VehicleDebug debug;
    {
      std::unique_lock<std::mutex> lck(mutex_);
      ecar_cmd = ecar_chassis_msg_.cmd_msg;
      ecar_stat = ecar_chassis_msg_.stat_msg;
      traj = trajectory_;
    }

    vehicle_manager_->ControlTask(&trajectory_, &ecar_stat, &ecar_cmd);
    vehicle_manager_->SimTask(&ecar_stat, &ecar_cmd, &debug);

    {
      std::unique_lock<std::mutex> lck(mutex_);
      ecar_chassis_msg_.cmd_msg = ecar_cmd;
      vehicle_debug_ = debug;
    }
  }

  void Start()
  {
    if (false == is_inited_.load())
    {
      logger_->Log(nirvana::common::log::Log_Error, "Vehicle node start failed !");
      return;
    }

    if (task_manager_->StartAll() == false)
    {
      logger_->Log(nirvana::common::log::Log_Error, "task manager start failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "task manager start all tasks successfully.");

    logger_->Log(nirvana::common::log::Log_Info, "Vehicle node start successfully.");
  }

  void Stop()
  {
    task_manager_->StopAll();
    logger_->Log(nirvana::common::log::Log_Info, "Vehicle node stopped.");
  }

private:
  std::shared_ptr<nirvana::common::time::ITime> timer_;
  std::shared_ptr<nirvana::common::log::ILog> logger_;
  std::atomic<bool> is_inited_;
  std::atomic<bool> is_message_manager_online_;
  std::shared_ptr<VehicleManager> vehicle_manager_;
  std::shared_ptr<nirvana::common::communication::IpcManager> ipc_manager_;
  std::shared_ptr<nirvana::common::schedule::ThreadManager> task_manager_;
  std::mutex mutex_;
  ::message::msg::EcarChassis ecar_chassis_msg_;
  ::vehicle::msg::Trajectory trajectory_;
  ::vehicle::msg::VehicleDebug vehicle_debug_;
};

// message::ecar_chassis CanbusNode::ecar_chassis_data = message::ecar_chassis();
