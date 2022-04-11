#pragma once

#include <string>
#include <deque>
#include <atomic>
#include <mutex>
#include <thread>
#include <functional>
// #include <any.h>
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include "../ISubscriber.h"
using std::placeholders::_1;

namespace nirvana
{
  namespace common
  {
    namespace communication
    {
      template <typename DataT>
      class RosSubscriber : public ISubscriber
      {
      public:
        RosSubscriber()
            : is_sub_thread_started(false) {}
        virtual ~RosSubscriber()
        {
          is_sub_thread_started = false;
          if (sub_thread_.joinable())
            sub_thread_.join();
        }
        virtual bool Init(void* node, std::string topic, int buff_size, int buff_num) override;
        virtual void Start() override;
        virtual int GetDataQueue(int queue_size, void *const data) override;
        virtual void GetCurrentData(void *const data) override;
        virtual std::string GetErrorString() override;

      private:
        bool is_sub_thread_started;
        void SubThreadTask();
        void MsgCallback(const std::shared_ptr<DataT> data);
        void ErrorLog(std::string err_str);

      private:
        // ros::NodeHandle nh_;
        rclcpp::Node* node_;
        std::shared_ptr<common::log::ILog> logger_;
        // ros::Subscriber subscriber_;
        typename std::shared_ptr<rclcpp::Subscription<DataT>> subscriber_;
        std::thread sub_thread_;
        mutable std::mutex mutex_;
        std::atomic<int> buff_num_;
        std::deque<DataT> data_;
        std::string err_str_;
      };

      template <typename DataT>
      bool RosSubscriber<DataT>::Init(void* node, std::string topic, int buff_size, int buff_num)
      {
        if(node == nullptr)
        {
          ErrorLog("RosSubscriber init failed, node is nullptr !");
          return false;
        }
        node_ = (rclcpp::Node*)node;

        buff_num_.exchange(buff_num);        
        // subscriber_ = nh_.subscribe(topic, buff_size, &RosSubscriber::MsgCallback, this);
        // subscriber_ = node_->create_subscription<DataT>(topic, std::bind(&RosSubscriber::MsgCallback, this, _1), buff_size);
        subscriber_ = node_->create_subscription<DataT>(topic, rclcpp::QoS(rclcpp::KeepLast(0)), std::bind(&RosSubscriber::MsgCallback, this, _1));
        return true;
      }

      template <typename DataT>
      void RosSubscriber<DataT>::Start()
      {
        is_sub_thread_started = true;
        sub_thread_ = std::thread(&RosSubscriber<DataT>::SubThreadTask, this);
      }

      template <typename DataT>
      int RosSubscriber<DataT>::GetDataQueue(int queue_size, void *const data)
      {
        buff_num_.exchange(queue_size);
        std::deque<DataT> *data_queue = (std::deque<DataT> *)data;
        std::unique_lock<std::mutex> lck(mutex_);
        if (data_.size() > 0)
        {
          data_queue->insert(data_queue->end(), data_.begin(), data_.end());
          data_.clear();
        }
        while (data_queue->size() > queue_size)
          data_queue->pop_front();

        return data_queue->size();
      }

      template <typename DataT>
      void RosSubscriber<DataT>::GetCurrentData(void *const data)
      {
        DataT *data_cur = (DataT *)data;
        std::unique_lock<std::mutex> lck(mutex_);
        if (data_.size() > 0)
          *data_cur = data_.back();
        data_.clear();
      }

      template <typename DataT>
      std::string RosSubscriber<DataT>::GetErrorString()
      {
        std::unique_lock<std::mutex> lck(mutex_);
        return err_str_;
      }

      //privates
      template <typename DataT>
      void RosSubscriber<DataT>::SubThreadTask()
      {
        // while (is_sub_thread_started)
        // {
        //   ros::spinOnce();
          
        //   std::this_thread::sleep_for(std::chrono::microseconds(2));
        // }
        // rclcpp::spin(std::shared_ptr<rclcpp::Node>(node_));
      }

      template <typename DataT>
      void RosSubscriber<DataT>::MsgCallback(const std::shared_ptr<DataT> data)
      {
        std::unique_lock<std::mutex> lck(mutex_);
        data_.push_back(*data);
        while (data_.size() > 0 && data_.size() > buff_num_.load())
        {
          data_.pop_front();
        }
      }

      template <typename DataT>
      void RosSubscriber<DataT>::ErrorLog(std::string err_str)
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
