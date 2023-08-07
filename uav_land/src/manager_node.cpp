#include "general.h"
#include "manager.h"
#include "rosClient.h"

Manager principal;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "manager_control");
  ros::NodeHandle *nh = new ros::NodeHandle();
  ros::Rate loop_rate(20);

  ROSClient ros_client(nh);
  principal.Init(&ros_client, 5, 1);

  while(ros::ok()){
    ros::spinOnce();
    principal.print_parameters();
    principal.update();
    
    loop_rate.sleep();
  }
  delete nh;
}