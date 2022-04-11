#pragma once
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>
#include <atomic>

#include "../Itransform_listener.h"

namespace nirvana
{
  namespace common
  {
    namespace transform
    {
      class RosTransformListener : public ITransformListener
      {
      public:
        RosTransformListener()
            : inited_(false), frame_id_(""), child_frame_id_("") {}
        virtual ~RosTransformListener() {}
        virtual bool Init(void* node, 
                          std::string frame_id,
                          std::string child_frame_id,
                          std::shared_ptr<common::time::ITime> timer,
                          std::shared_ptr<common::log::ILog> logger);
        virtual bool Transform(enum Data2TransType type, const void *const src, void *const dest);

      private:
        rclcpp::Node* node_;
        std::shared_ptr<common::time::ITime> timer_;
        std::shared_ptr<common::log::ILog> logger_;
        std::atomic<bool> inited_;
        std::shared_ptr<tf2_ros::Buffer> transform_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
        mutable std::mutex mutex_;
        std::string frame_id_;
        std::string child_frame_id_;
      };
    }
  }
}