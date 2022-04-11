#pragma once

#include <string>
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include "../IPublisher.h"

namespace nirvana
{
  namespace common
  {
    namespace communication
    {
      template <typename DataT>
      class RosPublisher : public IPublisher
      {
      public:
        RosPublisher() {}
        virtual ~RosPublisher() {}

        virtual bool Init(void* node, std::string topic, int buff_size);
        virtual void Publish(void *const data);
        virtual std::string GetErrorString() override;

      private:
        void ErrorLog(std::string err_str);

      private:
        // ros::NodeHandle nh_;
        // ros::Publisher publisher_;        
        rclcpp::Node* node_;
        std::shared_ptr<rclcpp::Publisher<DataT>> publisher_;

        mutable std::mutex mutex_;
        std::string err_str_;
      };

      template <typename DataT>
      bool RosPublisher<DataT>::Init(void* node, std::string topic, int buff_size)
      {
        if(node == nullptr)
        {
          ErrorLog("RosPublisher init failed, node is nullptr !");
          return false;
        }
        node_ = (rclcpp::Node*)node;

        // publisher_ = nh_.advertise<DataT>(topic, buff_size);
        publisher_ = node_->create_publisher<DataT>(topic, buff_size);
        return true;
      }

      template <typename DataT>
      void RosPublisher<DataT>::Publish(void *const data)
      {
        DataT *data_ptr = (DataT *)data;
        publisher_->publish(static_cast<DataT>(*data_ptr));
      }

      template <typename DataT>
      std::string RosPublisher<DataT>::GetErrorString()
      {
        std::unique_lock<std::mutex> lck(mutex_);
        return err_str_;
      }

      template <typename DataT>
      void RosPublisher<DataT>::ErrorLog(std::string err_str)
      {
        std::unique_lock<std::mutex> lck(mutex_);
        if(node_ != nullptr)
          err_str_ = node_->get_name() + ':' + ' ' + err_str;
        else
          err_str_ = err_str;
      }
    } // namespace communication
  }   // namespace common
} // namespace nirvana