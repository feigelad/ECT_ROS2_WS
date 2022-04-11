
#include "ros_transform_broadcaster.h"

namespace nirvana
{
  namespace common
  {
    namespace transform
    {
      bool RosTransformBroadcaster::Init(void *node,
                                         std::string frame_id,
                                         std::string child_frame_id,
                                         enum TfBroadcastType type,
                                         std::shared_ptr<common::time::ITime> timer,
                                         std::shared_ptr<common::log::ILog> logger)
      {
        if(node == nullptr)
          return false;
        node_ = (rclcpp::Node*)node;

        if (frame_id == "" || child_frame_id == "")
          return false;
        frame_id_ = frame_id;
        child_frame_id_ = child_frame_id;
        transform_type_ = type;
        if (timer == nullptr)
          return false;
        timer_ = timer;

        if (logger == nullptr)
          return false;
        logger_ = logger;

        static_broadcaster_ = std::shared_ptr<tf2_ros::StaticTransformBroadcaster>(new tf2_ros::StaticTransformBroadcaster(node_));
        broadcaster_ = std::shared_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster(node_));

        inited_.exchange(true);
        logger_->Log(nirvana::common::log::Log_Info, child_frame_id + " to " + frame_id + " broadcast init successful.");
        return true;
      }

      void RosTransformBroadcaster::Broadcast(double stamp, float pos_x, float pos_y, float pos_z, float att_r, float att_p, float att_y)
      {
        if (inited_.load() == false)
          return;
        std::string frame_id = "";
        std::string child_frame_id = "";
        enum TfBroadcastType broadcast_type;
        {
          std::unique_lock<std::mutex> lck(mutex_);
          frame_id = frame_id_;
          child_frame_id = child_frame_id_;
          broadcast_type = transform_type_;
        }

        geometry_msgs::msg::TransformStamped transformStamped;
        builtin_interfaces::msg::Time tm;
        transformStamped.header.stamp = tm.set__sec(stamp);
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame_id;
        transformStamped.transform.translation.x = pos_x;
        transformStamped.transform.translation.y = pos_y;
        transformStamped.transform.translation.z = pos_z;
        tf2::Quaternion q;
        q.setRPY(att_r, att_p, att_y);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        if (broadcast_type == Static_Transform_Broadcast)
        {
          static_broadcaster_->sendTransform(transformStamped);
          logger_->Log(nirvana::common::log::Log_Info, child_frame_id + " to " + frame_id + " static broadcast successful.");
        }
        else
          broadcaster_->sendTransform(transformStamped);
      }

      void RosTransformBroadcaster::Broadcast(double stamp, float pos_x, float pos_y, float pos_z, float quat_x, float quat_y, float quat_z, float quat_w)
      {
        if (inited_.load() == false)
          return;
        std::string frame_id = "";
        std::string child_frame_id = "";
        enum TfBroadcastType broadcast_type;
        {
          std::unique_lock<std::mutex> lck(mutex_);
          frame_id = frame_id_;
          child_frame_id = child_frame_id_;
          broadcast_type = transform_type_;
        }

        geometry_msgs::msg::TransformStamped transformStamped;
        builtin_interfaces::msg::Time tm;
        transformStamped.header.stamp = tm.set__sec(stamp);
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame_id;
        transformStamped.transform.translation.x = pos_x;
        transformStamped.transform.translation.y = pos_y;
        transformStamped.transform.translation.z = pos_z;
        transformStamped.transform.rotation.x = quat_x;
        transformStamped.transform.rotation.y = quat_y;
        transformStamped.transform.rotation.z = quat_z;
        transformStamped.transform.rotation.w = quat_w;
        if (broadcast_type == Static_Transform_Broadcast)
        {
          static_broadcaster_->sendTransform(transformStamped);
          logger_->Log(nirvana::common::log::Log_Info, child_frame_id + " to " + frame_id + " static broadcast successful.");
        }
        else
          broadcaster_->sendTransform(transformStamped);
      }
    }
  }
}