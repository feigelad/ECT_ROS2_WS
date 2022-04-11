#pragma once

#include "ISubscriber.h"
#include "ros_subscriber/ros_subscriber.h"

namespace nirvana
{
  namespace common
  {
    namespace communication
    {
      class SubscriberFactory
      {
      public:
        template <typename DataT>
        ISubscriber *CreateRosSubscriber(void* node, std::string topic, int buff_size);
      };

      template <typename DataT>
      ISubscriber *SubscriberFactory::CreateRosSubscriber(void* node, std::string topic, int buff_size)
      {
        ISubscriber *sub = new RosSubscriber<DataT>();
        sub->Init(node, topic, buff_size, 1);
        sub->Start();
        return sub;
      }
    } // namespace communication
  }   // namespace common
} // namespace nirvana
