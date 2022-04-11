#include <proj_api.h>
#include "rtk_ins570d.h"
#include "../../../common/quaternion/quaternion_base.h"
#include "message/msg/ecar_chassis_stat.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nirvana
{
  namespace location
  {
    void Rtk_Ins570d::Parse(const void *const chass_stat, void *const pose)
    {
      ::message::msg::EcarChassisStat *stat = (::message::msg::EcarChassisStat *)chass_stat;
      geometry_msgs::msg::PoseStamped *pose_res = (geometry_msgs::msg::PoseStamped *)pose;
      pose_res->header.stamp = stat->imu_stat_msg.ins570d_pos1_stat_msg.header.stamp;
      pose_res->header.frame_id = "world_coord_base";
      //position
      UTMCoor xy;
      double lat = stat->imu_stat_msg.ins570d_pos2_stat_msg.imu_rtkpos_latitude.value;
      double lon = stat->imu_stat_msg.ins570d_pos2_stat_msg.imu_rtkpos_longitude.value;
      double hei = stat->imu_stat_msg.ins570d_pos1_stat_msg.imu_rtkpos_height.value;
      double roll_rad = stat->imu_stat_msg.ins570d_attitude_stat_msg.imu_rtkatt_roll.value * PI / 180;
      double pitch_rad = stat->imu_stat_msg.ins570d_attitude_stat_msg.imu_rtkatt_pitch.value * PI / 180;
      double yaw_rad = stat->imu_stat_msg.ins570d_attitude_stat_msg.imu_rtkatt_yaw.value * PI / 180;

      // BLH2XYZ(lat, lon, hei, pose_res->position.x, pose_res->position.y, pose_res->position.z);
      int zone = static_cast<int>(lon / 6) + 31;
      // LatLonToUTMXY(31.4, 121.4, zone, xy);
      wgs84ToUTM(lat, lon, xy.x, xy.y);
      Eigen::Quaterniond q = euler2Quaternion(roll_rad, pitch_rad, yaw_rad);

      pose_res->pose.position.x = xy.x;
      pose_res->pose.position.y = xy.y;
      pose_res->pose.position.z = hei;
      pose_res->pose.orientation.x = q.x();
      pose_res->pose.orientation.y = q.y();
      pose_res->pose.orientation.z = q.z();
      pose_res->pose.orientation.w = q.w();

      //velocity
      // pose_res->velocity.

      //TODO: need to complete, lifei, 2020/12/12
    }
  } // namespace location
} // namespace nirvana