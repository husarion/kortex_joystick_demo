#ifndef KORTEX_JOYSTICK_DEMO_KORTEX_JOYSTICK_NODE_HPP_
#define KORTEX_JOYSTICK_DEMO_KORTEX_JOYSTICK_NODE_HPP_

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <geometry_msgs/Twist.h>
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <kortex_driver/TwistCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>

namespace kortex_joystick
{
const int LEFT_STICK_LR_ID = 0;
const int LEFT_STICK_UD_ID = 1;
const int RIGHT_STICK_LR_ID = 2;
const int RIGHT_STICK_UD_ID = 3;
const int CROSS_KEY_LR_ID = 4;
const int CROSS_KEY_UD_ID = 5;

const int X_BUTTON_ID = 0;
const int A_BUTTON_ID = 1;
const int B_BUTTON_ID = 2;
const int Y_BUTTON_ID = 3;
const int LB_BUTTON_ID = 4;
const int RB_BUTTON_ID = 5;
const int LT_BUTTON_ID = 6;
const int RT_BUTTON_ID = 7;
const int BACK_BUTTON_ID = 8;
const int START_BUTTON_ID = 9;

const int TWIST_LINEAR_MODE = 0;
const int TWIST_ANGULAR_MODE = 1;

const int GRIPPER_OPEN = 0;
const int GRIPPER_CLOSE = 1;

const int MANIPULATOR_READY = 9;  // In state no. 9 manipulator is ready to use

class KortexJoystickNode
{
public:
  KortexJoystickNode(ros::NodeHandle * nh, ros::NodeHandle * private_nh);

private:
  void feedback_callback(const kortex_driver::BaseCyclic_Feedback & feedback_msg);
  void joy_callback(const sensor_msgs::Joy & joy_msg);

  void joy_to_linear_twist_command(
    const sensor_msgs::Joy & joy_msg, kortex_driver::TwistCommand & twist_command_msg);

  void joy_to_angular_twist_command(
    const sensor_msgs::Joy & joy_msg, kortex_driver::TwistCommand & twist_command_msg);

  void call_gripper_controller_service(const bool state);

  float controll_speed(const float min_vel, const float max_vel, float vel, const float sign);

  ros::Subscriber feedback_sub_;
  ros::Subscriber joy_sub_;
  ros::Publisher kortex_twist_pub_;
  ros::Publisher emergency_stop_pub_;
  ros::Publisher clear_faults_pub_;
  ros::Publisher gripper_controller_pub_;

  typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperControllerClient;
  GripperControllerClient gripper_controller_client_;

  int active_state_;
  int mode_;
  int gripper_state_;
  float max_linear_vel_;
  float min_linear_vel_;
  float max_angular_vel_;
  float min_angular_vel_;
  float linear_vel_;
  float angular_vel_;
  float gripper_closed_position_;
  std::string node_name_;
};

};  // namespace kortex_joystick

#endif  // KORTEX_JOYSTICK_DEMO_KORTEX_JOYSTICK_NODE_HPP_