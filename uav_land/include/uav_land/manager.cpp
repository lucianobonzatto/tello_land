#include "manager.h"

Manager::Manager()
{
}

Manager::~Manager()
{
}

void Manager::Init(ROSClient *drone_control,
                   double joyLinearVelocity,
                   double joyAngularVelocity)
{
  joy_linear_velocity = joyLinearVelocity;
  joy_angular_velocity = joyAngularVelocity;
}

void Manager::print_parameters()
{
  cout << "================" << endl;
  // cout << "\ttrack: " << track.header.stamp << endl;
  cout << "\tpose: " << pose << endl;
  cout << "\tjoy: " << joy.header.stamp << endl;
  cout << "\todom: " << odom.header.stamp << endl;
  cout << "\tstate: " << states_name[state_machine.get_state()] << endl;
  follow_controller.print_parameters();
  land_controller.print_parameters();
}

void Manager::update()
{
  FOLLOW_CONTROL_action();
  STATES state = state_machine.get_state();
  switch (state)
  {
  case STATES::STOPPED:
    STOPPED_action();
    break;
  case STATES::TAKE_OFF:
    TAKE_OFF_action();
    break;
  case STATES::LAND:
    LAND_action();
    break;
  case STATES::JOY_CONTROL:
    JOY_CONTROL_action();
    break;
  case STATES::LAND_CONTROL:
    LAND_CONTROL_action();
    break;
  case STATES::FOLLOW_CONTROL:
    FOLLOW_CONTROL_action();
    break;
  default:
    break;
  }

  if (!parameters.data.empty())
  {
    follow_controller.update_parameters(&parameters.data[0]);
  }
  if (state_machine.update_state(joy))
  {
    send_velocity(0, 0, 0, 0);
  }
}

void Manager::STOPPED_action()
{
}

void Manager::TAKE_OFF_action()
{
  std_msgs::Empty emptyMsg;
  ROS_client->takeoff_pub.publish(emptyMsg);
}

void Manager::LAND_action()
{
  std_msgs::Empty emptyMsg;
  ROS_client->cmd_vel_pub.publish(emptyMsg);
}

void Manager::JOY_CONTROL_action()
{
  if (joy.header.stamp.isZero())
  {
    return;
  }
  if (joy_last_timestamp == joy.header.stamp)
  {
    send_velocity(0, 0, 0, 0);
    return;
  }
  joy_last_timestamp = joy.header.stamp;

  send_velocity(joy.axes[1] * joy_linear_velocity,
                joy.axes[0] * joy_linear_velocity,
                joy.axes[4] * joy_linear_velocity,
                joy.axes[3] * joy_angular_velocity);
}

void Manager::LAND_CONTROL_action()
{
  geometry_msgs::Twist velocity;
  velocity = land_controller.get_velocity();
  send_velocity(velocity);
}

void Manager::FOLLOW_CONTROL_action()
{
  geometry_msgs::Twist velocity;
  velocity = follow_controller.get_velocity(pose);
  send_velocity(velocity);
}

void Manager::send_velocity(double x_linear, double y_linear, double z_linear, double angular)
{
  geometry_msgs::Twist velocity;
  velocity.linear.x = x_linear;
  velocity.linear.y = y_linear;
  velocity.linear.z = z_linear;

  velocity.angular.x = 0;
  velocity.angular.y = 0;
  velocity.angular.z = angular;
  ROS_client->cmd_vel_pub.publish(velocity);
}

void Manager::send_velocity(geometry_msgs::Twist velocity)
{
  ROS_client->cmd_vel_pub.publish(velocity);
}

void Manager::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  pose = *msg;
}

void Manager::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  odom = *msg;
}

void Manager::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  joy = *msg;
}

void Manager::parametersCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
  parameters = *msg;
}

void Manager::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  imu = *msg;
}

void Manager::statusCallback(const tello_driver::TelloStatus::ConstPtr &msg)
{
  status = *msg;
}
