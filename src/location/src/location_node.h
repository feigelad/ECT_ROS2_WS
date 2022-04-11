#pragma once

#include <thread>

// #include "ros/ros.h"
// #include "dynamic_reconfigure/server.h"

#include <rclcpp/rclcpp.hpp>
#include "../../common/syslog/log_factory.h"
#include "../../common/communication/ipc_manager/ipc_manager.h"
#include "../../common/schedule/thread_manager/thread_manager.h"
// #include "location/pose.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "message/msg/ecar_chassis.hpp"
#include "../../location/rtk/rtk_factory.h"

//test
#include "../../common/transform/broadcaster/transform_broadcaster_factory.h"
#include "../../common/transform/listener/transform_listener_factory.h"

#include <deque>

class LocationNode : public rclcpp::Node
{
public:
  LocationNode()
    : Node("location_node") {}
  virtual ~LocationNode() {}
  void onInit()
  {
    //Init data
    is_inited_.exchange(false);

    //Get Systime Instance
    std::shared_ptr<nirvana::common::time::ITime> timer = std::shared_ptr<nirvana::common::time::ITime>(nirvana::common::time::SysTime::GetInstance());
    if (timer == nullptr)
      return;
    timer_ = timer;

    //Create logger
    nirvana::common::log::LogFactory log_factory;
    logger_ = std::shared_ptr<nirvana::common::log::ILog>(log_factory.CreateRosLogger());
    logger_->Init(this, timer);

    //Create Ipc Manager Instance
    ipc_manager_ = std::shared_ptr<nirvana::common::communication::IpcManager>(new nirvana::common::communication::IpcManager());
    if (false == ipc_manager_->Init(this, logger_))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Ipc manager init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Create ipc manager instance successfully.");

    //Register Ipc Messages
    if (false == ipc_manager_->RegistMsg<::message::msg::EcarChassis>("EcarChassisMsg", nirvana::common::communication::MsgSub))
    {
      logger_->Log(nirvana::common::log::Log_Error, "EcarChassisMsg regist failed !");
      return;
    }
    if (false == ipc_manager_->RegistMsg<geometry_msgs::msg::PoseStamped>("PoseMsg", nirvana::common::communication::MsgPub))
    {
      logger_->Log(nirvana::common::log::Log_Error, "PoseMsg regist failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Regist ipc messages successfully.");

    ipc_manager_->Start();

    //Rtk
    nirvana::location::RtkFactory rtk_factory;
    rtk_location_ = std::shared_ptr<nirvana::location::Rtk>(rtk_factory.CreateRtkIns570d());
    if (rtk_location_ == nullptr)
    {
      logger_->Log(nirvana::common::log::Log_Error, "Rtk instance create failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Create rtk successfully.");

    //Transform
    nirvana::common::transform::TransformBroadcasterFactory tbsf;
    world2vehicle_transform_broadcaster_ = std::shared_ptr<nirvana::common::transform::ITransformBroadcaster>(tbsf.CreateTransformBroadcaster());
    if (world2vehicle_transform_broadcaster_ == nullptr || false == world2vehicle_transform_broadcaster_->Init(this, "world_coord_base", "vehicle_coord_base", nirvana::common::transform::Dynamic_Transform_Broadcast, timer, logger_))
    {
      logger_->Log(nirvana::common::log::Log_Error, "transform broadcaster instance create failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Create transform broadcaster instance successfully.");

    //Task Manager
    task_manager_ = std::shared_ptr<nirvana::common::schedule::ThreadManager>(new nirvana::common::schedule::ThreadManager());
    if (task_manager_->Init("location_node", timer_, logger_, ipc_manager_) == false)
    {
      logger_->Log(nirvana::common::log::Log_Error, "Task manager init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Task manager init successfully.");

    //Regist task threads
    if (false == task_manager_->RegistTaskThread("pub_thread", std::bind(&LocationNode::PubTsk, this), 0.02, 0.05, 0.1))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Task manager regist pub_thread failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Task manager regist pub_thread successfully.");

    if (false == task_manager_->RegistTaskThread("sub_thread", std::bind(&LocationNode::SubTsk, this), 0.01, 0.02, 0.05))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Task manager regist sub_thread failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Task manager regist sub_thread successfully.");

    is_inited_.exchange(true);
    logger_->Log(nirvana::common::log::Log_Info, "Location node init successfully.");
  }

public:
  // static void PubThread(LocationNode *_p)
  // {
  //   _p->PubTsk();
  // }

  void PubTsk()
  {
    std::unique_lock<std::mutex> lck(mutex_);
    pose_msg_.header.frame_id = GetFrameId();
    // pose_msg_.header.seq++;
    rclcpp::Clock clock;
    pose_msg_.header.stamp = clock.now();
    if (false == ipc_manager_->PubMsg("PoseMsg", &pose_msg_))
    {
      logger_->Log(nirvana::common::log::Log_Error, "IpcManager Pub message failed !");
    }
    world2vehicle_transform_broadcaster_->Broadcast(timer_->Now2Sec(), pose_msg_.pose.position.x, pose_msg_.pose.position.y, pose_msg_.pose.position.z, pose_msg_.pose.orientation.x, pose_msg_.pose.orientation.y, pose_msg_.pose.orientation.z, pose_msg_.pose.orientation.w);
  }

  // static void SubThread(LocationNode *_p)
  // {
  //   _p->SubTsk();
  // }

  void SubTsk()
  {
    std::deque<::message::msg::EcarChassis> cmd_queue;
    if (0 > ipc_manager_->SubMsg("EcarChassisMsg", 1, &cmd_queue))
    {
      logger_->Log(nirvana::common::log::Log_Error, "IpcManager Sub message failed !");
    }
    if (false == cmd_queue.empty())
    {
      std::unique_lock<std::mutex> lck(mutex_);
      ecar_chassis_msg_ = cmd_queue.back();
      rtk_location_->Parse(&ecar_chassis_msg_.stat_msg, &pose_msg_);

      // std::cout << std::setprecision(15) << "pose_x:" << pose_.position.x << "   pos_y:" << pose_.position.y << "   pos_z:" << pose_.position.z << std::endl;
    }
  }

  void Start()
  {
    if (task_manager_->StartAll() == false)
    {
      logger_->Log(nirvana::common::log::Log_Error, "task manager start failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "task manager start all tasks successfully.");
  }

  void Stop()
  {
    task_manager_->StopAll();
  }

private:
  std::string GetFrameId()
  {
    return "world_coord_base";
  }

private:
  std::shared_ptr<nirvana::common::time::ITime> timer_;
  std::shared_ptr<nirvana::common::log::ILog> logger_;
  std::shared_ptr<nirvana::common::communication::IpcManager> ipc_manager_;
  std::shared_ptr<nirvana::location::Rtk> rtk_location_;
  std::shared_ptr<nirvana::common::transform::ITransformBroadcaster> world2vehicle_transform_broadcaster_;
  std::shared_ptr<nirvana::common::schedule::ThreadManager> task_manager_;
  std::atomic<bool> is_inited_;
  std::mutex mutex_;
  ::message::msg::EcarChassis ecar_chassis_msg_;
  geometry_msgs::msg::PoseStamped pose_msg_;
};
