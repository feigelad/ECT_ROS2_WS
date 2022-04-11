#include <cstdio>
#include "location_node.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<LocationNode>  location_node = std::shared_ptr<LocationNode>(new LocationNode());
  location_node->onInit();
  location_node->Start();

  while (rclcpp::ok())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  location_node->Stop();

  return 0;
}
