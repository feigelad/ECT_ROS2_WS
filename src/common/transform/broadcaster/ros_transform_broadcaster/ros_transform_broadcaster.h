#pragma once
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
// #include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mutex>
#include <atomic>

#include "../Itransform_broadcaster.h"

using std::placeholders::_1;

namespace nirvana
{
  namespace common
  {
    namespace transform
    {
      class RosTransformBroadcaster : public ITransformBroadcaster
      {
      public:
        RosTransformBroadcaster()
            : inited_(false), frame_id_(""), child_frame_id_("") {}
        virtual ~RosTransformBroadcaster() {}
        virtual bool Init(void *node,
                          std::string frame_id,
                          std::string child_frame_id,
                          enum TfBroadcastType type,
                          std::shared_ptr<common::time::ITime> timer,
                          std::shared_ptr<common::log::ILog> logger);
        virtual void Broadcast(double stamp, float pos_x, float pos_y, float pos_z, float att_r, float att_p, float att_y);
        virtual void Broadcast(double stamp, float pos_x, float pos_y, float pos_z, float quat_x, float quat_y, float quat_z, float quat_w);

        // private:
        //   void StaticBroadcast(const geometry_msgs::TransformStamped &tf_stamped);
        //   void DynamicBroadcast(const geometry_msgs::TransformStamped &tf_stamped);

      private:
        rclcpp::Node* node_;
        std::shared_ptr<common::time::ITime> timer_;
        std::shared_ptr<common::log::ILog> logger_;
        std::atomic<bool> inited_;
        // tf2_ros::StaticTransformBroadcaster
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
        mutable std::mutex mutex_;
        enum TfBroadcastType transform_type_;
        std::string frame_id_;
        std::string child_frame_id_;
      };
    }
  }
}