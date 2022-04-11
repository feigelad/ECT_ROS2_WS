#include <cstdio>
#include "vehicle_node.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<VehicleNode>  vehicle_node = std::shared_ptr<VehicleNode>(new VehicleNode());
  vehicle_node->onInit();
  vehicle_node->Start();

  while (rclcpp::ok())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  vehicle_node->Stop();

  return 0;
}
