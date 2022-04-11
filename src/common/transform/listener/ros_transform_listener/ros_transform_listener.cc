
#include "ros_transform_listener.h"
namespace nirvana
{
  namespace common
  {
    namespace transform
    {
      bool RosTransformListener::Init(void* node,
                                      std::string frame_id,
                                      std::string child_frame_id,
                                      std::shared_ptr<common::time::ITime> timer,
                                      std::shared_ptr<common::log::ILog> logger)
      {
        if (node == nullptr)
          return false;
        node_ = (rclcpp::Node*)node;

        if (frame_id == "" || child_frame_id == "")
          return false;
        frame_id_ = frame_id;
        child_frame_id_ = child_frame_id;
        if (timer == nullptr)
          return false;
        timer_ = timer;

        if (logger == nullptr)
          return false;
        logger_ = logger;

        transform_buffer_ = std::shared_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(node_->get_clock()));
        transform_listener_ = std::shared_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(*transform_buffer_, node_));

        inited_.exchange(true);
        logger_->Log(nirvana::common::log::Log_Info, child_frame_id + " to " + frame_id + " listener init successful.");
        return true;
      }

      bool RosTransformListener::Transform(enum Data2TransType type, const void *const src, void *const dest)
      {
        if (inited_.load() == false)
          return false;
        std::string frame_id = "";
        std::string child_frame_id = "";
        {
          std::unique_lock<std::mutex> lck(mutex_);
          frame_id = frame_id_;
          child_frame_id = child_frame_id_;
        }
        geometry_msgs::msg::TransformStamped child2base = transform_buffer_->lookupTransform(frame_id, child_frame_id, tf2::TimePoint(), tf2::Duration(20000000));
        switch (type)
        {
        case PointTransform:
        {
          // transform_buffer_.transform(*((geometry_msgs::Point *)src), *((geometry_msgs::Point *)dest), frame_id, child_frame_id);
          tf2::doTransform(*(geometry_msgs::msg::Point *)src, *(geometry_msgs::msg::Point *)dest, child2base);
          break;
        }
        case PointStampedTransform:
        {
          transform_buffer_->transform(*(geometry_msgs::msg::PointStamped *)src, *(geometry_msgs::msg::PointStamped *)dest, frame_id);
          break;
        }
        case Vector3Transform:
        {
          // transform_buffer_.transform(*(geometry_msgs::Vector3 *)src, *(geometry_msgs::Vector3 *)dest, frame_id);
          tf2::doTransform(*(geometry_msgs::msg::Vector3 *)src, *(geometry_msgs::msg::Vector3 *)dest, child2base);
          break;
        }
        case Vector3StampedTransform:
        {
          transform_buffer_->transform(*(geometry_msgs::msg::Vector3Stamped *)src, *(geometry_msgs::msg::Vector3Stamped *)dest, frame_id);
          break;
        }
        case PoseTransform:
        {
          // transform_buffer_.transform(*(geometry_msgs::Pose *)src, *(geometry_msgs::Pose *)dest, frame_id);
          tf2::doTransform(*(geometry_msgs::msg::Pose *)src, *(geometry_msgs::msg::Pose *)dest, child2base);
          break;
        }
        case QuaternionTransform:
        {
          // transform_buffer_.transform(*(geometry_msgs::Quaternion *)src, *(geometry_msgs::Quaternion *)dest, frame_id);
          tf2::doTransform(*(geometry_msgs::msg::Quaternion *)src, *(geometry_msgs::msg::Quaternion *)dest, child2base);
          break;
        }
        case QuaternionStampedTransform:
        {
          transform_buffer_->transform(*(geometry_msgs::msg::QuaternionStamped *)src, *(geometry_msgs::msg::QuaternionStamped *)dest, frame_id);
          break;
        }
        default:
        {
          logger_->Log(nirvana::common::log::Log_Error, "Transform type error !");
          break;
        }
        }
        return true;
      }

      // void RosTransformListener::Lookup(double stamp, float &pos_x, float &pos_y, float &pos_z, float &att_r, float &att_p, float &att_y)
      // {
      //   std::string frame_id = "";
      //   std::string child_frame_id = "";
      //   {
      //     std::unique_lock<std::mutex> lck(mutex_);
      //     frame_id = frame_id_;
      //     child_frame_id = child_frame_id_;
      //   }
      //   if (frame_id == "" || child_frame_id == "")
      //   {
      //     logger_->Log(common::log::Log_Error, frame_id + " listener init failed !");
      //     return;
      //   }

      //   ros::Time tm;
      //   geometry_msgs::TransformStamped transformStamped;
      //   transformStamped = transform_buffer_.lookupTransform(child_frame_id, frame_id, tm.fromSec(stamp));
      //   tf2::Quaternion q;
      //   q.setX(transformStamped.transform.rotation.x);
      //   q.setY(transformStamped.transform.rotation.y);
      //   q.setZ(transformStamped.transform.rotation.z);
      //   q.setW(transformStamped.transform.rotation.w);

      //   tf2::Vector3 vec = q.getAxis();
      //   pos_x = transformStamped.transform.translation.x;
      //   pos_y = transformStamped.transform.translation.y;
      //   pos_z = transformStamped.transform.translation.z;
      //   att_r = vec[0];
      //   att_p = vec[1];
      //   att_y = vec[2];
      // }
    }
  }
}