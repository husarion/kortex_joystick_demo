#include "kortex_joystick_demo/kortex_joystick_node.hpp"

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "kortex_joystick");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  kortex_joystick::KortexJoystickNode kortex_joystick_node(&nh, &private_nh);

  try {
    ros::spin();
  } catch (const std::exception & e) {
    ROS_ERROR("[%s] Error: %s", ros::this_node::getName().c_str(), e.what());
  }

  ros::shutdown();
  return 0;
}