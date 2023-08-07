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
  // drone_connection = drone_control;
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
  // drone_connection->offboardMode();
  // drone_connection->takeOff();
}

void Manager::LAND_action()
{
  // drone_connection->land();
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
  // drone_connection->cmd_vel(x_linear, y_linear, z_linear, angular);
}

void Manager::send_velocity(geometry_msgs::Twist velocity)
{
  // drone_connection->cmd_vel(velocity.linear.x,
  //                           velocity.linear.y,
  //                           velocity.linear.z,
  //                           velocity.angular.z);
}

void Manager::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  // principal.set_pose(*msg);
}
void Manager::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  // principal.set_odom(*msg);
}
void Manager::joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
  // principal.set_joy(*msg);
}
void Manager::parametersCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
  // principal.set_parameters(*msg);
}