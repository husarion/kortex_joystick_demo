#include "kortex_joystick_demo/kortex_joystick_node.hpp"

namespace kortex_joystick
{
KortexJoystickNode::KortexJoystickNode(ros::NodeHandle * nh, ros::NodeHandle * private_nh)
: gripper_controller_client_("gripper_controller/gripper_cmd", true)
{
  private_nh->param<float>("max_linear_vel", max_linear_vel_, 0.2);
  private_nh->param<float>("min_linear_vel", min_linear_vel_, 0.05);
  private_nh->param<float>("max_angular_vel", max_angular_vel_, 0.5);
  private_nh->param<float>("min_angular_vel", min_angular_vel_, 0.05);
  private_nh->param<float>("gripper_closed_position", gripper_closed_position_, 0.8);

  node_name_ = ros::this_node::getName();
  node_name_ = node_name_.substr(1, node_name_.length());

  // set initial mode
  mode_ = TWIST_LINEAR_MODE;

  // set initial speed to the middle of the speed range
  linear_vel_ = min_linear_vel_ + (max_linear_vel_ - min_linear_vel_) / 2;
  angular_vel_ = min_angular_vel_ + (max_angular_vel_ - min_angular_vel_) / 2;

  error_sub_ = nh->subscribe("/kinova_gen3/kortex_error", 1, &KortexJoystickNode::error_callback, this);
  joy_sub_ = nh->subscribe("joy", 1, &KortexJoystickNode::joy_callback, this);
  kortex_twist_pub_ = nh->advertise<kortex_driver::TwistCommand>("in/cartesian_velocity", 1);
  emergency_stop_pub_ = nh->advertise<std_msgs::Empty>("in/emergency_stop", 1);
  clear_faults_pub_ = nh->advertise<std_msgs::Empty>("in/clear_faults", 1);

  ROS_INFO("[%s] Node started", node_name_.c_str());
}

void KortexJoystickNode::error_callback(const kortex_driver::KortexError & error_msg)
{
  error_code_ = error_msg.code;
  ROS_INFO("[%s] ========= Error code: ", error_code_.c_str());
}

void KortexJoystickNode::joy_callback(const sensor_msgs::Joy & joy_msg)
{
  // emergency stop pressed
  if (joy_msg.buttons.at(B_BUTTON_ID)) {
    emergency_stop_pub_.publish(std_msgs::Empty());
    ROS_WARN("[%s] Emergency stop pressed", node_name_.c_str());
  }

  // driving robot
  if (joy_msg.buttons.at(LB_BUTTON_ID)) {
    emergency_stop_pub_.publish(std_msgs::Empty());
    return;
  }

  // clear faults
  if (joy_msg.buttons.at(A_BUTTON_ID)) {
    ROS_INFO("[%s] Clearing faults", node_name_.c_str());
    clear_faults_pub_.publish(std_msgs::Empty());
  }

  // switch to twist linear mode
  if (joy_msg.buttons.at(X_BUTTON_ID) && mode_ != TWIST_LINEAR_MODE) {
    mode_ = TWIST_LINEAR_MODE;
    ROS_INFO("[%s] Using twist linear mode", node_name_.c_str());
  }

  // switch to twist angular mode
  if (joy_msg.buttons.at(Y_BUTTON_ID) && mode_ != TWIST_ANGULAR_MODE) {
    mode_ = TWIST_ANGULAR_MODE;
    ROS_INFO("[%s] Using twist angular mode", node_name_.c_str());
  }

  // control speed +-10%
  if (joy_msg.axes.at(CROSS_KEY_UD_ID) != 0.0) {
    if (mode_ == TWIST_LINEAR_MODE) {
      linear_vel_ = controll_speed(
        min_linear_vel_, max_linear_vel_, linear_vel_, joy_msg.axes.at(CROSS_KEY_UD_ID));
      ROS_INFO("[%s] Increasing linear velocity to: %f", node_name_.c_str(), linear_vel_);
    } else if (mode_ == TWIST_ANGULAR_MODE) {
      angular_vel_ = controll_speed(
        min_angular_vel_, max_angular_vel_, angular_vel_, joy_msg.axes.at(CROSS_KEY_UD_ID));
      ROS_INFO("[%s] Increasing angular velocity to: %f", node_name_.c_str(), angular_vel_);
    }
  }

  // close gripper
  if (joy_msg.buttons.at(LT_BUTTON_ID)) {
    ROS_INFO("[%s] Closing gripper", node_name_.c_str());
    call_gripper_controller_service(GRIPPER_CLOSE);
  }

  // open gripper
  if (joy_msg.buttons.at(RT_BUTTON_ID)) {
    ROS_INFO("[%s] Opening gripper", node_name_.c_str());
    call_gripper_controller_service(GRIPPER_OPEN);
  }

  kortex_driver::TwistCommand kortex_twist_msg;

  if (mode_ == TWIST_LINEAR_MODE) {
    joy_to_linear_twist_command(joy_msg, kortex_twist_msg);
    kortex_twist_pub_.publish(kortex_twist_msg);
  } else if (mode_ == TWIST_ANGULAR_MODE) {
    joy_to_angular_twist_command(joy_msg, kortex_twist_msg);
    kortex_twist_pub_.publish(kortex_twist_msg);
  }
}

void KortexJoystickNode::joy_to_linear_twist_command(
  const sensor_msgs::Joy & joy_msg, kortex_driver::TwistCommand & twist_command_msg)
{
  float linear_velocity_factor = 0.1;
  auto a = joy_msg.axes.at(0);
  twist_command_msg.twist.linear_x = linear_vel_ * joy_msg.axes.at(LEFT_STICK_LR_ID);
  twist_command_msg.twist.linear_y = - linear_vel_ * joy_msg.axes.at(LEFT_STICK_UD_ID);
  twist_command_msg.twist.linear_z = linear_vel_ * joy_msg.axes.at(RIGHT_STICK_UD_ID);
}

void KortexJoystickNode::joy_to_angular_twist_command(
  const sensor_msgs::Joy & joy_msg, kortex_driver::TwistCommand & twist_command_msg)
{
  float linear_velocity_factor = 0.1;
  auto a = joy_msg.axes.at(0);
  twist_command_msg.twist.angular_x = angular_vel_ * joy_msg.axes.at(LEFT_STICK_UD_ID);
  twist_command_msg.twist.angular_y = - angular_vel_ * joy_msg.axes.at(LEFT_STICK_LR_ID);
  twist_command_msg.twist.angular_z = angular_vel_ * joy_msg.axes.at(RIGHT_STICK_UD_ID);
}

void KortexJoystickNode::call_gripper_controller_service(const bool state)
{
  try {
    control_msgs::GripperCommandGoal goal;

    if (state == GRIPPER_OPEN) {
      goal.command.position = 0.0;
      gripper_controller_client_.sendGoal(goal);
      gripper_controller_client_.waitForResult(ros::Duration(30));
    } else if (state == GRIPPER_CLOSE) {
      goal.command.position = gripper_closed_position_;
      gripper_controller_client_.sendGoal(goal);
      gripper_controller_client_.waitForResult(ros::Duration(30));
    }
    ROS_INFO("[%s] Gripper operation finished", node_name_.c_str());

  } catch (const std::exception & e) {
    ROS_ERROR("[%s] Error calling gripper action: %s", node_name_.c_str(), e.what());
  }
}

float KortexJoystickNode::controll_speed(
  const float min_vel, const float max_vel, float vel, const float sign)
{
  vel = vel + (sign * vel * 0.1);
  if (vel < min_vel) {
    vel = min_vel;
  }
  if (vel > max_vel) {
    vel = max_vel;
  }
  return vel;
}

}  // namespace kortex_joystick