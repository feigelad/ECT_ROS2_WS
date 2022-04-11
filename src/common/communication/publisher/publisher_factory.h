#pragma once

#include "IPublisher.h"
#include "ros_publisher/ros_publisher.h"

namespace nirvana
{
  namespace common
  {
    namespace communication
    {
      class PublisherFactory
      {
      public:
        template <typename DataT>
        IPublisher *CreateRosPublisher(void* node, std::string topic, int buff_size);
      };

      template <typename DataT>
      IPublisher *PublisherFactory::CreateRosPublisher(void* node, std::string topic, int buff_size)
      {
        IPublisher *pub = new RosPublisher<DataT>();
        pub->Init(node, topic, buff_size);
        return pub;
      }
    } // namespace communication
  }   // namespace common
} // namespace nirvana