// #include <typeinfo>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "ros_systime.h"



namespace nirvana
{
  namespace common
  {
    namespace time
    {
      uint64_t RosSysTime::Now2MSec()
      {
        rclcpp::Clock clock;
        rclcpp::Time now = clock.now();
        return static_cast<uint64_t>(now.nanoseconds() / 1000000.0);
      }

      double RosSysTime::Now2Sec()
      {
        rclcpp::Clock clock;
        rclcpp::Time now = clock.now();
        return now.seconds();
      }

      void RosSysTime::FillTimeStamp(void *const stamp)
      {
        rclcpp::Time *stamp_ptr = (rclcpp::Time *)stamp;
        rclcpp::Clock clock;
        *stamp_ptr = clock.now();
      }
    } // namespace time
  }   // namespace common
} // namespace nirvana