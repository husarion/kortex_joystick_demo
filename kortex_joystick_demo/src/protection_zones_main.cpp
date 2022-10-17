#include "kortex_joystick_demo/protection_zones_node.hpp"
#include <signal.h>

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "protection_zones", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ProtectionZonesNode protection_zones_node(&nh, &private_nh);

  try {
    ros::spin();
  } catch (const std::exception & e) {
    ROS_ERROR("[%s] Error: %s", ros::this_node::getName().c_str(), e.what());
  }

  // ros::shutdown();
  return 0;
}