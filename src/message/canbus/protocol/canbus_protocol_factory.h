#pragma once

#include "ecar_standard/brake_stat_0x211.h"
#include "ecar_standard/brake_diag_0x212.h"
#include "ecar_standard/park_stat_0x213.h"
#include "ecar_standard/park_diag_0x214.h"
#include "ecar_standard/steer_stat_0x215.h"
#include "ecar_standard/steer_diag_0x216.h"
#include "ecar_standard/throt_stat_0x217.h"
#include "ecar_standard/throt_diag_0x218.h"
#include "ecar_standard/body_stat_0x219.h"
#include "ecar_standard/body_diag_0x220.h"
#include "ecar_standard/anticrsh_stat_0x221.h"
#include "ecar_standard/bat_stat_0x222.h"
#include "ecar_standard/bat_diag_0x223.h"
#include "ecar_standard/veh_stat_0x240.h"
#include "ecar_standard/veh_dyna_stat_0x250.h"
#include "ecar_standard/front_wheel_speed_0x251.h"
#include "ecar_standard/rear_wheel_speed_0x252.h"
#include "ecar_standard/total_trip_stat_0x254.h"
#include "ecar_standard/remote_trip_stat_0x255.h"
#include "ecar_standard/automatic_trip_stat_0x256.h"
#include "ecar_standard/cloud_trip_stat_0x257.h"
#include "ecar_standard/power_stat_0x260.h"
#include "ecar_standard/remoter_stat1_0x270.h"
#include "ecar_standard/remoter_stat2_0x271.h"
#include "ecar_standard/remoter_stat3_0x272.h"
#include "ecar_standard/veh_info_0x2a0.h"
#include "ecar_standard/brake_cmd_0x111.h"
#include "ecar_standard/park_cmd_0x112.h"
#include "ecar_standard/steer_cmd_0x113.h"
#include "ecar_standard/throt_cmd_0x114.h"
#include "ecar_standard/body_cmd_0x115.h"
#include "ecar_standard/power_cmd_0x117.h"
#include "ecar_standard/cloud_drive_cmd_0x118.h"
#include "ecar_standard/cloud_body_cmd_0x119.h"
//imu m2
#include "imu_m2/week_time_stat_0x701.h"
#include "imu_m2/utc_time_stat_0x702.h"
#include "imu_m2/rtk_pos1_stat_0x703.h"
#include "imu_m2/rtk_pos2_stat_0x704.h"
#include "imu_m2/rtk_speed_stat_0x705.h"
#include "imu_m2/rtk_attitude_stat_0x706.h"
#include "imu_m2/rtk_pos_cov_stat_0x707.h"
#include "imu_m2/rtk_speed_cov_stat_0x708.h"
#include "imu_m2/rtk_att_cov_stat_0x709.h"
#include "imu_m2/rtk_status_0x710.h"
#include "imu_m2/imu_gyro1_stat_0x721.h"
#include "imu_m2/imu_gyro2_stat_0x722.h"
#include "imu_m2/imu_gyro3_stat_0x723.h"
#include "imu_m2/imu_input_cmd_0x099.h"

//imu ins570d
#include "imu_ins570d/imu_acc_0x500.h"
#include "imu_ins570d/imu_gyro_0x501.h"
#include "imu_ins570d/imu_attitude_0x502.h"
#include "imu_ins570d/imu_pos1_0x503.h"
#include "imu_ins570d/imu_pos2_0x504.h"
#include "imu_ins570d/imu_speed_0x505.h"
#include "imu_ins570d/imu_datainfo_0x506.h"
#include "imu_ins570d/imu_stdcov_0x507.h"

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      // enum CanProtocol
      class CanbusProtocolFactory
      {
      public:
        CanbusProtocol *CreateCanbusProtocol(uint32_t id);
      };
    } // namespace canbus
  }   // namespace message
} // namespace nirvana