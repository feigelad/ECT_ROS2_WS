#pragma once

#include <thread>

// #include "ros/ros.h"
// #include "dynamic_reconfigure/server.h"
#include <rclcpp/rclcpp.hpp>
// #include "../../common/syslog/log_factory.h"
#include "../../common/syslog/log_factory.h"
#include "../../message/canbus/can_dev/candev_factory.h"
#include "../../message/canbus/protocol/canbus_protocol_factory.h"
#include "../../message/canbus/message_manager/canbus_message_manager.h"
#include "../../common/communication/ipc_manager/ipc_manager.h"
#include "../../common/schedule/thread_manager/thread_manager.h"
#include "message/msg/ecar_chassis.hpp"

#include <deque>

using namespace nirvana::message::canbus;

class CanbusNode : public rclcpp::Node
{
public:
  CanbusNode()
  :Node("canbus_node"){}
  virtual ~CanbusNode()
  {
  }
  void onInit()
  {
    //data init
    is_inited_.exchange(false);
    //Get Systime Instance
    timer_ = std::shared_ptr<nirvana::common::time::ITime>(nirvana::common::time::SysTime::GetInstance());

    //Create logger
    nirvana::common::log::LogFactory log_factory;
    logger_ = std::shared_ptr<nirvana::common::log::ILog>(log_factory.CreateRosLogger());
    logger_->Init(this, timer_);

    //Create can api instance
    nirvana::message::canbus::CanDevFctory candev_factory;
    std::shared_ptr<ICan>
        can_api = std::shared_ptr<ICan>(candev_factory.CreateZlgCan());
    canbus_error_t err_can = can_api->Init(logger_, 3, 0, 0);
    if (err_can != CAN_RESULT_OK)
    {
      logger_->Log(nirvana::common::log::Log_Error, CanBusError::GetCanbusErrorString(err_can) + "Dev Type: 3");
      // ROS_ERROR_STREAM(CanBusError::GetCanbusErrorString(err_can) << "Dev Type: 3");
      return;
    }

    logger_->Log(nirvana::common::log::Log_Info, "Can api instance create and init successfully.");
    // ROS_INFO("Can api instance create and init successfully.");

    //Create canbus message manager instance
    canbus_message_manager = std::shared_ptr<CanbusMessageManager>(new CanbusMessageManager());
    err_can = canbus_message_manager->Init(logger_, timer_, can_api, false, false);
    if (err_can != CAN_RESULT_OK)
    {
      logger_->Log(nirvana::common::log::Log_Error, CanBusError::GetCanbusErrorString(err_can));
      // ROS_ERROR_STREAM(CanBusError::GetCanbusErrorString(err_can));
      return;
    }

    logger_->Log(nirvana::common::log::Log_Info, "Canbus message manager instance create and init successfully.");
    // ROS_INFO("Canbus message manager instance create and init successfully.");

    //Register signals and protocols here
    /** Protocols Definition=======================================================
      *  LiFei, 2020/7/7
      * ------------------------------CDCU Recv-----------------------------------||
      *  @SN |   @MsgId  | @MsgName              | @Ide | @Period(ms) | @Send/Recv||
      *   1       0x211    CDCU_BrakeStatus          0      20             Recv   ||
      *   2       0x212    CDCU_BrakeDiag            0      100            Recv   ||
      *   3       0x213    CDCU_ParkStatus           0      20             Recv   ||
      *   4       0x214    CDCU_ParkDiag             0      100            Recv   ||
      *   5       0x215    CDCU_SteerStatus          0      20             Recv   ||
      *   6       0x216    CDCU_SteerDiag            0      100            Recv   ||
      *   7       0x217    CDCU_DriveStatus          0      20             Recv   ||
      *   8       0x218    CDCU_DriveDiag            0      100            Recv   ||
      *   9       0x219    CDCU_BodyStatus           0      20             Recv   ||
      *   10      0x220    CDCU_BodyDiag             0      100            Recv   ||
      *   11      0x221    CDCU_AntiCrshStatus       0      20             Recv   ||
      *   12      0x222    CDCU_BatStatus            0      100            Recv   ||
      *   13      0x223    CDCU_BatDiag              0      100            Recv   ||
      *   14      0x240    CDCU_VehState             0      100            Recv   ||
      *   15      0x250    CDCU_VehDyncState         0      20             Recv   ||
      *   16      0x251    CDCU_VehFtWhlSpd          0      20             Recv   ||
      *   17      0x252    CDCU_VehRrWhlSpd          0      20             Recv   ||
      *   18      0x260    CDCU_PowerStatus          0      100            Recv   ||
      *   19      0x270    CDCU_RemoteInfo1          0      100            Recv   ||
      *   20      0x271    CDCU_RemoteInfo2          0      100            Recv   ||
      *   21      0x254    CDCU_TotalTripMeter       0      1000           Recv   ||
      *   22      0x255    CDCU_RmtDrvTripMeter      0      1000           Recv   ||
      *   23      0x256    CDCU_AutoDrvTripMeter     0      1000           Recv   ||
      *   24      0x257    CDCU_CldDrvTripMeter      0      1000           Recv   ||
      *   25      0x2A0    CDCU_VehicleVIN           0      1000           Recv   ||
      * -----------------------------CDCU Send------------------------------------||
      *   1       0x111    ADCU_BrakeCmd             0      20             Send   ||
      *   2       0x112    ADCU_ParkCmd              0      20             Send   ||
      *   3       0x113    ADCU_SteerCmd             0      20             Send   ||
      *   4       0x114    ADCU_DriveCmd             0      20             Send   ||
      *   5       0x115    ADCU_BodyCmd              0      100            Send   ||
      *   6       0x117    ADCU_PowerCmd             0      100            Send   ||
      *   7       0x118    ADCU_CldDrvCmd            0      20             Send   ||
      *   8       0x119    ADCU_CldBodyCmd           0      20             Send   ||
      *   9       0x11A    ADCU_CldPowerCmd          0      20             Send   ||
      * -----------------------------IMU Ins570d Recv-----------------------------||
      *   1       0x500    IMU_Acc_Status            0      20           MSG_RECV ||
      *   2       0x501    IMU_Gyro_Status           0      20           MSG_RECV ||
      *   3       0x502    IMU_Attitude_Status       0      20           MSG_RECV ||
      *   4       0x503    IMU_Position1_Status      0      20           MSG_RECV ||
      *   5       0x504    IMU_Position2_Status      0      20           MSG_RECV ||
      *   6       0x505    IMU_Speed_Status          0      20           MSG_RECV ||
      *   7       0x506    IMU_DataInfo_Status       0      20           MSG_RECV ||
      *   8       0x507    IMU_StdCov_Status         0      20           MSG_RECV ||
      *==========================================================================**/
    nirvana::message::canbus::CanbusProtocolFactory protocol_factory;

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x211)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x211 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x212)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x212 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x213)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x213 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x214)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x214 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x215)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x215 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x216)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x216 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x217)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x217 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x218)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x218 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x219)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x219 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x220)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x220 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x221)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x221 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x222)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x222 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x223)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x223 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x240)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x240 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x250)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x250 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x251)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x251 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x252)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x252 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x260)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x260 regist failed !");
      return;
    }

    //    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x270)), &ecar_chassis_msg_))
    {
        //TODO: Register protocol fault, LiFei, 2020/7/8
        // logger_->Log(nirvana::common::log::Log_Error, "Frame 0x270 regist failed !");
        // return;
    }

    //    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x271)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      // logger_->Log(nirvana::common::log::Log_Error, "Frame 0x271 regist failed !");
      // return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x254)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x254 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x255)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x255 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x256)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x256 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x257)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x257 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x2A0)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x2A0 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x270)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x270 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x271)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x271 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x272)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x272 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x111)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x111 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x112)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x112 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x113)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x113 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x114)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x114 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x115)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x115 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x117)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x117 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x118)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x118 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x119)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x119 regist failed !");
      return;
    }

    //    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x11A)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      // logger_->Log(nirvana::common::log::Log_Error, "Frame 0x11A regist failed !");
      // return;
    }

    //IMU
    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x701)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x701 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x702)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x702 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x703)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x703 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x704)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x704 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x705)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x705 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x706)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x706 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x707)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x707 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x708)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x708 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x709)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x709 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x710)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x710 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x721)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x721 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x722)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x722 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x723)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x723 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x099)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x099 regist failed !");
      return;
    }

    //Imu Ins570d
    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x500)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x500 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x501)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x501 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x502)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x502 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x503)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x503 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x504)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x504 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x505)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x505 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x506)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x506 regist failed !");
      return;
    }

    if (CAN_RESULT_OK != canbus_message_manager->RegistProtocol(std::shared_ptr<CanbusProtocol>(protocol_factory.CreateCanbusProtocol(0x507)), &ecar_chassis_msg_))
    {
      //TODO: Register protocol fault, LiFei, 2020/7/8
      logger_->Log(nirvana::common::log::Log_Error, "Frame 0x507 regist failed !");
      return;
    }

    //Create Ipc Manager Instance
    ipc_manager_ = std::shared_ptr<nirvana::common::communication::IpcManager>(new nirvana::common::communication::IpcManager());
    if (false == ipc_manager_->Init(this,logger_))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Ipc manager init failed !");
      return;
    }

    //Register Ipc Messages
    if (false == ipc_manager_->RegistMsg<::message::msg::EcarChassisCmd>("EcarChassisCmdMsg", nirvana::common::communication::MsgSub))
    {
      logger_->Log(nirvana::common::log::Log_Error, "EcarChassisCmdMsg regist failed !");
      return;
    }
    if (false == ipc_manager_->RegistMsg<::message::msg::EcarChassis>("EcarChassisMsg", nirvana::common::communication::MsgPub))
    {
      logger_->Log(nirvana::common::log::Log_Error, "EcarChassisMsg regist failed !");
      return;
    }

    ipc_manager_->Start();

    //Task Manager
    task_manager_ = std::shared_ptr<nirvana::common::schedule::ThreadManager>(new nirvana::common::schedule::ThreadManager());
    if (task_manager_->Init("canbus_node", timer_, logger_, ipc_manager_) == false)
    {
      logger_->Log(nirvana::common::log::Log_Error, "Task manager init failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Task manager init successfully.");

    //Regist task threads
    if (false == task_manager_->RegistTaskThread("canrecv_thread", std::bind(&CanbusNode::CanMsgRecvTsk, this), 0.01, 0.02, 0.5))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Task manager regist canrecv_thread failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Task manager regist canrecv_thread successfully.");

    if (false == task_manager_->RegistTaskThread("cansend_thread", std::bind(&CanbusNode::CanMsgSendTsk, this), 0.01, 0.02, 0.05))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Task manager regist cansend_thread failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Task manager regist cansend_thread successfully.");

    if (false == task_manager_->RegistTaskThread("pub_thread", std::bind(&CanbusNode::PubTsk, this), 0.02, 0.05, 0.1))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Task manager regist pub_thread failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Task manager regist pub_thread successfully.");

    if (false == task_manager_->RegistTaskThread("sub_thread", std::bind(&CanbusNode::SubTsk, this), 0.01, 0.02, 0.05))
    {
      logger_->Log(nirvana::common::log::Log_Error, "Task manager regist sub_thread failed !");
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Task manager regist sub_thread successfully.");

    logger_->Log(nirvana::common::log::Log_Info, "CanbusNode inited successfully .");
    is_inited_.exchange(true);
  }

public:
  void CanMsgRecvTsk()
  {
    std::unique_lock<std::mutex> lck(mutex_);
    // double stamp = timer_->Now2Sec();
    canbus_message_manager->CanbusRecvMsgFunc(&ecar_chassis_msg_.stat_msg);
    // logger_->Log(nirvana::common::log::Log_Info, "can recv task need " + std::to_string(timer_->Now2Sec() - stamp) + " secs .");
  }

  void CanMsgSendTsk()
  {
    std::unique_lock<std::mutex> lck(mutex_);
    // double stamp = timer_->Now2Sec();
    canbus_message_manager->CanbusSendMsgFunc(&ecar_chassis_msg_.cmd_msg);
    // logger_->Log(nirvana::common::log::Log_Info, "can send task need " + std::to_string(timer_->Now2Sec() - stamp) + " secs .");
  }

  void PubTsk()
  {
    std::unique_lock<std::mutex> lck(mutex_);
    if (false == ipc_manager_->PubMsg("EcarChassisMsg", &ecar_chassis_msg_))
    {
      logger_->Log(nirvana::common::log::Log_Error, "IpcManager Pub message failed !");
    }

    // ipc_manager_->PubMsg("Test", )
  }

  void SubTsk()
  {
    std::deque<::message::msg::EcarChassisCmd> cmd_queue;
    if (0 > ipc_manager_->SubMsg("EcarChassisCmdMsg", 1, &cmd_queue))
    {
      logger_->Log(nirvana::common::log::Log_Error, "IpcManager Sub message failed !");
    }
    if (false == cmd_queue.empty())
    {
      std::unique_lock<std::mutex> lck(mutex_);
      ecar_chassis_msg_.cmd_msg = cmd_queue.back();
    }
  }

  void Start()
  {
    if (false == is_inited_.load())
    {
      logger_->Log(nirvana::common::log::Log_Error, "Canbus node start failed !");
      return;
    }
    //Start Canbus message manager
    canbus_error_t err_can = canbus_message_manager->Start();
    if (err_can != CAN_RESULT_OK)
    {
      logger_->Log(nirvana::common::log::Log_Error, "" + CanBusError::GetCanbusErrorString(err_can));
      // ROS_ERROR_STREAM(CanBusError::GetCanbusErrorString(err_can));
      return;
    }
    logger_->Log(nirvana::common::log::Log_Info, "Canbus message manager start successfully.");
    // ROS_INFO("Canbus message manager start successfully.");

    if (task_manager_->StartAll() == false)
    {
      logger_->Log(nirvana::common::log::Log_Error, "task manager start failed !");
      return;
    }

    logger_->Log(nirvana::common::log::Log_Info, "task manager start all tasks successfully.");

    logger_->Log(nirvana::common::log::Log_Info, "Canbus node start successfully.");
  }

  void Stop()
  {
    task_manager_->StopAll();

    canbus_message_manager->Stop();
    logger_->Log(nirvana::common::log::Log_Info, "Canbus node stopped.");
  }

private:
  std::shared_ptr<nirvana::common::time::ITime> timer_;
  std::shared_ptr<nirvana::common::log::ILog> logger_;
  std::atomic<bool> is_inited_;
  std::shared_ptr<CanbusMessageManager> canbus_message_manager;
  std::shared_ptr<nirvana::common::communication::IpcManager> ipc_manager_;
  std::shared_ptr<nirvana::common::schedule::ThreadManager> task_manager_;
  std::mutex mutex_;
  ::message::msg::EcarChassis ecar_chassis_msg_;
};

// message::ecar_chassis CanbusNode::ecar_chassis_data = message::ecar_chassis();