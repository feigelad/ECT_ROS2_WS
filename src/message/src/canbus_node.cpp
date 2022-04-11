#include <cstdio>
#include "canbus_node.h"

// void CanbusNode::onInit()
  


  // void CanbusNode::CanMsgRecvTsk()
  

  // void CanbusNode::CanMsgSendTsk()
  

  // void CanbusNode::PubTsk()
  

  // void CanbusNode::SubTsk()
  

  // void CanbusNode::Start()
  

  // void CanbusNode::Stop()
  

int main(int argc, char ** argv)
{  
  rclcpp::init(argc, argv);

  std::shared_ptr<CanbusNode> canbus_node = std::shared_ptr<CanbusNode>(new CanbusNode());
  canbus_node->onInit();
  canbus_node->Start();

  while (rclcpp::ok())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  canbus_node->Stop();
  return 0;
}
